#!/bin/bash

# Archivo: /usr/local/bin/ptp-automation.sh
# Script para automatizar el proceso PTP en Jetson

# Configuración
INTERFACE="eth0"
PTP4L_CONFIG="/etc/linuxptp/ptp4l.conf"
LOG_FILE="/var/log/ptp-automation.log"

# Función para logging
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Función para limpiar procesos al salir
cleanup() {
    log_message "Deteniendo procesos PTP..."
    pkill -f "ptp4l.*$INTERFACE"
    pkill -f "phc2sys.*$INTERFACE"
    exit 0
}

# Capturar señales para limpieza
trap cleanup SIGTERM SIGINT

log_message "Iniciando automatización PTP en $INTERFACE"

# Verificar que la interfaz existe
if ! ip link show "$INTERFACE" &>/dev/null; then
    log_message "ERROR: La interfaz $INTERFACE no existe"
    exit 1
fi

# Verificar que el archivo de configuración existe
if [ ! -f "$PTP4L_CONFIG" ]; then
    log_message "ERROR: El archivo de configuración $PTP4L_CONFIG no existe"
    exit 1
fi

# Paso 1: Inicializar ptp4l
log_message "Iniciando ptp4l en $INTERFACE..."
ptp4l -i "$INTERFACE" -m -f "$PTP4L_CONFIG" &
PTP4L_PID=$!

# Verificar que ptp4l se inició correctamente
sleep 2
if ! kill -0 $PTP4L_PID 2>/dev/null; then
    log_message "ERROR: ptp4l no se pudo iniciar"
    exit 1
fi

log_message "ptp4l iniciado correctamente (PID: $PTP4L_PID)"

# Paso 2: Detectar flujo de red en eth0
log_message "Monitoreando flujo de red en $INTERFACE..."

NETWORK_DETECTED=false
TIMEOUT=60  # Timeout en segundos
START_TIME=$(date +%s)

while [ $NETWORK_DETECTED = false ]; do
    # Verificar actividad de red
    RX_PACKETS_BEFORE=$(cat /sys/class/net/$INTERFACE/statistics/rx_packets)
    sleep 2
    RX_PACKETS_AFTER=$(cat /sys/class/net/$INTERFACE/statistics/rx_packets)
    
    if [ $RX_PACKETS_AFTER -gt $RX_PACKETS_BEFORE ]; then
        NETWORK_DETECTED=true
        log_message "Flujo de red detectado en $INTERFACE"
    else
        # Verificar timeout
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))
        
        if [ $ELAPSED -gt $TIMEOUT ]; then
            log_message "ADVERTENCIA: Timeout esperando flujo de red. Continuando de todas formas..."
            NETWORK_DETECTED=true
        else
            log_message "Esperando flujo de red... ($ELAPSED/$TIMEOUT segundos)"
        fi
    fi
    
    # Verificar que ptp4l sigue ejecutándose
    if ! kill -0 $PTP4L_PID 2>/dev/null; then
        log_message "ERROR: ptp4l se detuvo inesperadamente"
        exit 1
    fi
done

# Paso 3: Iniciar phc2sys
log_message "Iniciando phc2sys..."
phc2sys -c "$INTERFACE" -s CLOCK_REALTIME -O 0 &
PHC2SYS_PID=$!

# Verificar que phc2sys se inició correctamente
sleep 2
if ! kill -0 $PHC2SYS_PID 2>/dev/null; then
    log_message "ERROR: phc2sys no se pudo iniciar"
    cleanup
    exit 1
fi

log_message "phc2sys iniciado correctamente (PID: $PHC2SYS_PID)"
log_message "Automatización PTP completada exitosamente"

# Mantener el script ejecutándose
while true; do
    # Verificar que ambos procesos siguen ejecutándose
    if ! kill -0 $PTP4L_PID 2>/dev/null; then
        log_message "ERROR: ptp4l se detuvo inesperadamente"
        cleanup
        exit 1
    fi
    
    if ! kill -0 $PHC2SYS_PID 2>/dev/null; then
        log_message "ERROR: phc2sys se detuvo inesperadamente"
        cleanup
        exit 1
    fi
    
    sleep 10
done