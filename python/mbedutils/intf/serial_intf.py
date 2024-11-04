from abc import ABCMeta, abstractmethod


class ISerial(ABCMeta):

    @abstractmethod
    def open(self) -> None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass

    @abstractmethod
    def read_all(self) -> bytes:
        pass

    @abstractmethod
    def write(self, data: bytes) -> None:
        pass

    @abstractmethod
    def flush(self) -> None:
        pass