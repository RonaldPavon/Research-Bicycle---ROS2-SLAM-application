# Imagen base oficial de Python
FROM python:3.10-slim

# Evita prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Instala dependencias del sistema
RUN apt-get update && apt-get install -y \
    python3-dev \
    gcc \
    libgpiod-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Instala dependencias de Python
RUN pip install --upgrade pip && \
    pip install \
    flask \
    flask-cors \
    docker==6.1.3 \
    requests==2.31.0 \
    urllib3==1.26.18 \
    Jetson.GPIO

# Agrega usuario para GPIO (opcional)
RUN groupadd -f -r gpio && \
    useradd -m -r -g gpio flaskuser

# Crea directorio de trabajo
WORKDIR /app

# Copia tu aplicación
COPY . .

# Establece el usuario (si quieres usar root por Jetson.GPIO, déjalo así)
USER root

# Expone el puerto de Flask
EXPOSE 5000

# Ejecuta tu backend
CMD ["python3", "bobbi-backend.py"]
