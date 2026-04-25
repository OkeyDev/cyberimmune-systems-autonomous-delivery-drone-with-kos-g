import json
import logging
from openai import AsyncOpenAI
from src.recognizers.base import BaseRecognizer, RecognitionResult
from src.core.config import settings

logger = logging.getLogger(__name__)

class VLLMRecognizer(BaseRecognizer):
    def __init__(self):
        self.client = AsyncOpenAI(
            base_url=settings.VLLM_BASE_URL,
            api_key=settings.VLLM_API_KEY
        )
        self.model = settings.VLLM_MODEL
        self.prompt = self._load_prompt()

    def _load_prompt(self) -> str:
        try:
            with open(settings.PROMPT_PATH, "r", encoding="utf-8") as f:
                return f.read().strip()
        except Exception as e:
            logger.error(f"Failed to load prompt from {settings.PROMPT_PATH}: {e}")
            return "Analyze the image and highlight any Latin letters and/or numbers it contains. Return as JSON array."

    async def recognize(self, image_base64: str) -> RecognitionResult:
        try:
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}"
                            },
                        },
                    ],
                }
            ]

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
            )

            content = response.choices[0].message.content
            if not content:
                return RecognitionResult(tag="NONE", rec_alt="NONE")

            start = content.rfind('[')
            end = content.rfind(']')
            
            results = []
            if start != -1 and end != -1 and end > start:
                json_part = content[start:end+1]
                try:
                    data = json.loads(json_part)
                    if isinstance(data, list):
                        results = [str(item) for item in data]
                except json.JSONDecodeError:
                    logger.warning(f"Failed to parse extracted JSON: {json_part}")
            
            if not results:
                try:
                    data = json.loads(content)
                    if isinstance(data, list):
                        results = [str(item) for item in data]
                except json.JSONDecodeError:
                    pass
                
            if not results:
                if not isinstance(results, list):
                    logger.warning(f"Unexpected JSON format from model: {content}")
                return RecognitionResult(tag="NONE", rec_alt="NONE")

            if len(results) == 1:
                return RecognitionResult(tag=results[0], rec_alt="NONE")
            else:
                return RecognitionResult(tag="NONE", rec_alt="1")

        except Exception as e:
            logger.error(f"Error during VLLM recognition: {e}")
            return RecognitionResult(tag="NONE", rec_alt="NONE")