from enum import Enum, unique
from typing import Tuple


@unique
class CapabilityCategory(str, Enum):
    TOPIC    = "topics"
    ACTION   = "actions"
    SERVICE  = "services"
    BUILTIN  = "builtins"

    @classmethod
    def ordered(cls) -> Tuple[str, ...]:
        """Return the canonical ordering that all helpers should follow."""
        return (
            cls.TOPIC.value,
            cls.ACTION.value,
            cls.SERVICE.value,
            cls.BUILTIN.value,
        )
