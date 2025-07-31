#!/bin/bash

# Cambiar por el nombre de tu hotspot
HOTSPOT_NAME="Hotspot"

# Activar el hotspot
nmcli connection up "$HOTSPOT_NAME"