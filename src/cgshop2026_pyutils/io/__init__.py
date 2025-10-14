import functools
from pathlib import Path

from ..schemas.instance import CGSHOP2026Instance
from ..schemas.solution import CGSHOP2026Solution


def open_file(func):
    """
    Decorator to open a file before calling the function and close it afterwards,
    if passed as string or pathlib.Path.
    """

    @functools.wraps(func)
    def wrapper(file, *args, **kwargs):
        if isinstance(file, str):
            file = Path(file)
        if isinstance(file, Path):
            with file.open() as f:
                return func(f, *args, **kwargs)
        return func(file, *args, **kwargs)

    return wrapper


@open_file
def read_instance(file) -> CGSHOP2026Instance:
    """
    Read an instance from a file.
    :param file: File object or path to the file.
    :return: Instance object
    """
    content = file.read()
    return CGSHOP2026Instance.model_validate_json(content)


@open_file
def read_solution(file) -> CGSHOP2026Solution:
    """
    Read a solution from a file.
    :param file: File object or path to the file.
    :return: Solution object
    """
    content = file.read()
    return CGSHOP2026Solution.model_validate_json(content)
