FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY jkbms2mqtt.py .

# Ejecuta como usuario no root
RUN useradd -m appuser
USER appuser

CMD ["python", "-u", "jkbms2mqtt.py"]
