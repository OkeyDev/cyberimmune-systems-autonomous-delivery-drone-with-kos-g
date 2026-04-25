import logging
from src.recognizers.base import BaseRecognizer, RecognitionResult

logger = logging.getLogger(__name__)

class StubRecognizer(BaseRecognizer):
    """
    A stub recognizer for testing purposes.
    Returns specific results based on the image parameter:
    - image=picture0 -> RecognitionResult(tag="NONE", rec_alt="1")
    - image=picture1 -> RecognitionResult(tag="A1", rec_alt="NONE")
    - image=picture2 -> RecognitionResult(tag="A2", rec_alt="NONE")
    - image=picture3 -> RecognitionResult(tag="A3", rec_alt="NONE")
    - image=picture4 -> RecognitionResult(tag="E1", rec_alt="NONE")
    - image=picture5 -> RecognitionResult(tag="E2", rec_alt="NONE")
    - image=picture6 -> RecognitionResult(tag="E3", rec_alt="NONE")
    - image=picture7 -> RecognitionResult(tag="E4", rec_alt="NONE")
    - image=picture8 -> RecognitionResult(tag="E5", rec_alt="NONE")
    - All other inputs -> RecognitionResult(tag="NONE", rec_alt="NONE")
    """

    MAPPING = {
        "picture0": ("NONE", "1"),
        "picture1": ("A1", "NONE"),
        "picture2": ("A2", "NONE"),
        "picture3": ("A3", "NONE"),
        "picture4": ("E1", "NONE"),
        "picture5": ("E2", "NONE"),
        "picture6": ("E3", "NONE"),
        "picture7": ("E4", "NONE"),
        "picture8": ("E5", "NONE"),
    }

    async def recognize(self, image_base64: str) -> RecognitionResult:
        """
        Recognize character pairs based on stub mapping.
        
        Args:
            image_base64: Image identifier string.
            
        Returns:
            RecognitionResult with tag and rec_alt fields.
        """
        if image_base64 in self.MAPPING:
            tag, rec_alt = self.MAPPING[image_base64]
            logger.info(f"Stub recognizer matched: {image_base64} -> tag={tag}, rec_alt={rec_alt}")
            return RecognitionResult(tag=tag, rec_alt=rec_alt)
        
        logger.info(f"Stub recognizer no match for: {image_base64} -> tag=NONE, rec_alt=NONE")
        return RecognitionResult(tag="NONE", rec_alt="NONE")