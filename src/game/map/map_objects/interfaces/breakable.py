import abc
from abc import ABC

class IBreakable(ABC):
    @abc.abstractmethod
    def break_it(self):
        raise NoneOverwriteError()

class NoneOverwriteError(Exception):
    pass
