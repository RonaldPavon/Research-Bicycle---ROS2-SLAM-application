#!/bin/bash

# Script de inicialización completa para Jetson
# Secuencia: WiFi → NTP sync → Desactivar NTP → Hotspot → PTP

LOG_FILE="/var/log/jetson-startup.log"

# CONFIGURACIÓN - CAMBIAR ESTOS VALORES
WIFI_CONNECTION="DLR Gastzugang"        # Nombre de tu conexión WiFi para internet
HOTSPOT_CONNECTION="Hotspot"       # Nombre de tu hotspot
ETH_INTERFACE="eth0"                  # Interfaz ethernet para PTP
PTP4L_CONFIG="/etc/linuxptp/ptp4l.conf"
NTP_TIMEOUT=30                        # Segundos para esperar sincronización NTP

# Función para logging
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Función para verificar si el tiempo está sincronizado
check_time_sync() {
    # Verificar si timedatectl muestra el tiempo como sincronizado
    if timedatectl status | grep -q "System clock synchronized: yes"; then
        return 0
    else
        return 1
    fi
}

log_message "=== INICIANDO SECUENCIA DE STARTUP JETSON ==="

# PASO 1: Conectar a WiFi para internet
log_message "PASO 1: Conectando a WiFi para sincronización..."

# Verificar que NetworkManager está activo
if ! systemctl is-active --quiet NetworkManager; then
    log_message "ERROR: NetworkManager no está activo"
    exit 1
fi

# Conectar a WiFi
log_message "Conectando a: $WIFI_CONNECTION"
nmcli radio wifi on
nmcli connection up "$WIFI_CONNECTION"

if [ $? -eq 0 ]; then
    log_message "WiFi conectado exitosamente"
else
    log_message "ERROR: No se pudo conectar a WiFi"
    exit 1
fi

# Esperar conectividad
log_message "Esperando conectividad a internet..."
sleep 5

# Verificar conectividad
if ping -c 3 8.8.8.8 >/dev/null 2>&1; then
    log_message "Conectividad a internet confirmada"
else
    log_message "ADVERTENCIA: Sin conectividad a internet, continuando..."
fi

# PASO 2: Sincronizar tiempo con NTP
log_message "PASO 2: Sincronizando tiempo con NTP..."

# Activar NTP si está desactivado
log_message "Activando sincronización NTP..."
timedatectl set-ntp true

# Forzar sincronización inmediata
systemctl restart systemd-timesyncd

# Esperar sincronización
log_message "Esperando sincronización de tiempo (máximo $NTP_TIMEOUT segundos)..."
SYNC_START=$(date +%s)

while ! check_time_sync; do
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - SYNC_START))
    
    if [ $ELAPSED -gt $NTP_TIMEOUT ]; then
        log_message "ADVERTENCIA: Timeout en sincronización NTP, continuando..."
        break
    fi
    
    log_message "Esperando sincronización... ($ELAPSED/$NTP_TIMEOUT segundos)"
    sleep 2
done

if check_time_sync; then
    log_message "Tiempo sincronizado correctamente: $(date)"
else
    log_message "ADVERTENCIA: Tiempo no sincronizado completamente, fecha actual: $(date)"
fi

# PASO 3: Desactivar NTP
log_message "PASO 3: Desactivando NTP..."
timedatectl set-ntp false

if [ $? -eq 0 ]; then
    log_message "NTP desactivado exitosamente"
else
    log_message "ERROR: No se pudo desactivar NTP"
fi

# PASO 4: Desconectar WiFi y activar hotspot
log_message "PASO 4: Activando hotspot..."

# Desconectar WiFi
log_message "Desconectando WiFi..."
nmcli connection down "$WIFI_CONNECTION"

# Esperar un momento
sleep 3

# Activar hotspot
log_message "Activando hotspot: $HOTSPOT_CONNECTION"
nmcli connection up "$HOTSPOT_CONNECTION"

if [ $? -eq 0 ]; then
    log_message "Hotspot activado exitosamente"
    
    # Verificar que está activo
    sleep 3
    if nmcli -t -f ACTIVE,NAME connection show | grep -q "yes:$HOTSPOT_CONNECTION"; then
        log_message "Hotspot confirmado como activo"
    else
        log_message "ERROR: Hotspot no se confirmó como activo"
    fi
else
    log_message "ERROR: No se pudo activar hotspot"
fi

# PASO 5: Inicializar PTP
log_message "PASO 5: Inicializando PTP..."

# Verificar que la interfaz ethernet existe
if ! ip link show "$ETH_INTERFACE" &>/dev/null; then
    log_message "ERROR: La interfaz $ETH_INTERFACE no existe"
    exit 1
fi

# Verificar archivo de configuración PTP
if [ ! -f "$PTP4L_CONFIG" ]; then
    log_message "ERROR: Archivo de configuración PTP no encontrado: $PTP4L_CONFIG"
    exit 1
fi

# Limpiar procesos PTP previos
pkill -f "ptp4l.*$ETH_INTERFACE" 2>/dev/null
pkill -f "phc2sys.*$ETH_INTERFACE" 2>/dev/null
sleep 2

# Iniciar ptp4l
log_message "Iniciando ptp4l en $ETH_INTERFACE..."
ptp4l -i "$ETH_INTERFACE" -m -f "$PTP4L_CONFIG" &
PTP4L_PID=$!

# Verificar que se inició
sleep 3
if kill -0 $PTP4L_PID 2>/dev/null; then
    log_message "ptp4l iniciado correctamente (PID: $PTP4L_PID)"
else
    log_message "ERROR: ptp4l no se pudo iniciar"
    exit 1
fi

# Detectar flujo de red (simplificado)
log_message "Esperando flujo de red en $ETH_INTERFACE..."
sleep 5  # Espera básica para estabilización

# Iniciar phc2sys
log_message "Iniciando phc2sys..."
phc2sys -c "$ETH_INTERFACE" -s CLOCK_REALTIME -O 0 &
PHC2SYS_PID=$!

# Verificar que se inició
sleep 3
if kill -0 $PHC2SYS_PID 2>/dev/null; then
    log_message "phc2sys iniciado correctamente (PID: $PHC2SYS_PID)"
else
    log_message "ERROR: phc2sys no se pudo iniciar"
fi

log_message "=== SECUENCIA DE STARTUP COMPLETADA ==="
log_message "Estado final:"
log_message "  - Tiempo sincronizado inicialmente: $(date)"
log_message "  - NTP: desactivado"
log_message "  - Hotspot: $HOTSPOT_CONNECTION activo"
log_message "  - PTP: ptp4l (PID: $PTP4L_PID) y phc2sys (PID: $PHC2SYS_PID)"

# Mantener el script corriendo para monitorear PTP
while true; do
    if ! kill -0 $PTP4L_PID 2>/dev/null; then
        log_message "ERROR: ptp4l se detuvo inesperadamente"
        exit 1
    fi
    
    if ! kill -0 $PHC2SYS_PID 2>/dev/null; then
        log_message "ERROR: phc2sys se detuvo inesperadamente"
        exit 1
    fi
    
    sleep 30
done