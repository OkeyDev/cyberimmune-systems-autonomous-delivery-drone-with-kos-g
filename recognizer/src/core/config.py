from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    # MQTT Configuration
    MQTT_HOST: str = "localhost"
    MQTT_PORT: int = 1883
    MQTT_USER: Optional[str] = None
    MQTT_PASSWORD: Optional[str] = None
    MQTT_INPUT_TOPIC: str = "api/image/request"
    MQTT_OUTPUT_TOPIC: str = "api/image/response"

    # VLLM Configuration
    VLLM_BASE_URL: str = "http://localhost:8000/v1"
    VLLM_API_KEY: str = "not-needed"
    VLLM_MODEL: str = "qwen3.5-4b"

    # App Configuration
    LOG_LEVEL: str = "INFO"
    PROMPT_PATH: str = "resources/char_recognition_prompt.txt"
    RECOGNIZER_TYPE: str = "vllm"  # Options: "vllm", "stub"

settings = Settings()