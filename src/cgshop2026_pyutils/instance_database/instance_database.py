import typing
import zipfile
from pathlib import Path

from ..schemas.instance import CGSHOP2026Instance



from .instance_file_database import InstanceFileDatabase
from .instance_zip_database import InstanceZipDatabase


class InstanceDatabase:
    """
    This class provides an interface to easily read instances from a folder or a zipfile
    where the instance files follow the naming convention 'instance-name.instance.json'.
    It supports subfolders but does not allow symbolic links.
    """

    def __init__(self, path: str, enable_cache: bool = False):
        """
        Initializes an InstanceDatabase that searches in a specified folder or zipfile for instances.
        :param path: Path to the folder or zipfile containing the instance files. The instance
                     files can be in subfolders, but their names must follow the pattern
                     NAME.instance.json.
        :param enable_cache: Whether to cache the loaded instances, which can consume significant memory.
        """
        self._inner_database = self._guess_database_class(path, enable_cache)

    def _guess_database_class(self, path: str, enable_cache: bool):
        """
        Determines whether the provided path refers to a folder or a zipfile, and returns the appropriate database class.
        :param path: Path to the folder or zipfile.
        :param enable_cache: Whether to cache the instances.
        :return: Instance of the appropriate database class.
        """
        path_obj = Path(path)

        if path_obj.is_dir():
            return InstanceFileDatabase(path, enable_cache=enable_cache)
        if path_obj.is_file():
            if zipfile.is_zipfile(path):
                return InstanceZipDatabase(path, enable_cache=enable_cache)
            msg = f"'{path}' is not a valid zipfile."
            raise FileNotFoundError(msg)
        msg = f"'{path}' is neither a directory nor a file."
        raise FileNotFoundError(msg)

    def __iter__(self) -> typing.Iterator[CGSHOP2026Instance]:
        """
        Iterates over all instances in the database.
        :return: An iterable of Cgshop2025Instance objects.
        """
        yield from self._inner_database

    def __getitem__(self, name: str) -> CGSHOP2026Instance:
        """
        Retrieves an instance by name, adjusting the name if it contains a path or file extension.
        :param name: The name of the instance to retrieve.
        :return: The Cgshop2025Instance object corresponding to the given name.
        :raises KeyError: If the instance is not found.
        """
        # Remove any path components
        name = Path(name).name

        # Strip the .json extension if present
        extension = self._inner_database.extension
        if name.endswith(extension):
            name = name[: -len(extension)]

        return self._inner_database[name]
