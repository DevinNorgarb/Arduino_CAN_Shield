FROM python:3.12-slim

WORKDIR /app

COPY docker/can-stream/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY docker/can-stream/extract_dashboard.py .
COPY src/web_dashboard.cpp /tmp/web_dashboard.cpp
RUN python extract_dashboard.py /tmp/web_dashboard.cpp /app/dashboard.html

COPY scripts/can_stream_server.py .
RUN mkdir -p /data

ENV CAN_STREAM_HOST=0.0.0.0 \
    CAN_STREAM_PORT=8765 \
    CAN_STREAM_OUT=/data/capture.ndjson \
    PYTHONUNBUFFERED=1

EXPOSE 8765

CMD ["python", "can_stream_server.py"]
