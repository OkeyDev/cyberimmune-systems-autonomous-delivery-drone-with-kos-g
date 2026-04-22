import logging
import asyncio
from aiomqtt import Client, Message
from src.recognizers.base import BaseRecognizer
from src.core.config import settings

logger = logging.getLogger(__name__)

class MQTTHandler:
    def __init__(self, recognizer: BaseRecognizer):
        self.recognizer = recognizer
        self.host = settings.MQTT_HOST
        self.port = settings.MQTT_PORT
        self.username = settings.MQTT_USER
        self.password = settings.MQTT_PASSWORD
        self.base_input_topic = settings.MQTT_INPUT_TOPIC
        self.base_output_topic = settings.MQTT_OUTPUT_TOPIC

    async def run(self):
        """
        Main loop for MQTT client: subscribe to the input topic and process messages.
        """
        while True:
            try:
                async with Client(
                    hostname=self.host,
                    port=self.port,
                    username=self.username,
                    password=self.password,
                    identifier="char-recognizer-service"
                ) as client:
                    logger.info(f"Connected to MQTT broker at {self.host}:{self.port}")
                    await client.subscribe(f"{self.base_input_topic}/#")
                    logger.info(f"Subscribed to {self.base_input_topic}/#")
                    
                    async for message in client.messages:
                        await self.process_message(client, message)
                            
            except Exception as e:
                logger.error(f"MQTT Client connection error: {e}. Retrying in 5 seconds...")
                await asyncio.sleep(5)

    async def process_message(self, client: Client, message: Message):
        """
        Handle a single incoming MQTT message.
        """
        try:
            # Extract ID from topic: api/image/request/{id}
            topic_str = str(message.topic)
            topic_parts = topic_str.split("/")
            if len(topic_parts) < 3:
                logger.warning(f"Received message on topic without ID: {message.topic}")
                return
            
            id_ = topic_parts[-1]
            
            payload = message.payload.decode("utf-8")
            
            image_base64 = None
            if payload.startswith("image="):
                image_base64 = payload[len("image="):]
            
            if not image_base64:
                logger.warning(f"Received message without image data. ID: {id_}")
                return

            logger.info(f"Processing image for ID: {id_}")
            
            result = await self.recognizer.recognize(image_base64)
            
            response_payload = f"result={result.tag}&rec_alt={result.rec_alt}"
            
            output_topic = f"{self.base_output_topic}/{id_}"
            await client.publish(
                output_topic,
                payload=response_payload
            )
            logger.info(f"Published result for ID: {id_} to {output_topic}: {response_payload}")

        except Exception as e:
            logger.error(f"Error while processing message: {e}")