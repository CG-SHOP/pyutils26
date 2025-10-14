import os
from pathlib import Path
import typing
import zipfile

from ..schemas.instance import CGSHOP2026Instance

from .instance_base_database import InstanceBaseDatabase


class InstanceZipDatabase(InstanceBaseDatabase):
    """
    This class allows to easily read instances from a zipfile if the instance files
    follow the naming convention 'instance-name.instance.json'. It allows subfolder
    but no symbolic links.
    """

    def __init__(self, path: str, enable_cache: bool = False):
        """
        Create an InstanceDatabase that searches in a specified zipfile for instances.
        :param path: Path to the zipfile that contains the instance files.
                        The instance files can be in subfolders but have the names have to be
                        NAME.instance.json.
        :param enable_cache: Should the loaded instances be cached? This can take quite
                        a lot of memory
        """

        super().__init__(path, enable_cache)
        self._zipfile = zipfile.ZipFile(path)

    def _find_path(self, name):
        for instance_path in self._zipfile.filelist:
            if self._filename_fits_name(
                os.path.split(instance_path.filename)[-1], name
            ):
                return instance_path
        msg = f"Did not find a suitable file for {name} in {self._path}"
        raise KeyError(msg)

    def __iter__(self) -> typing.Iterator[CGSHOP2026Instance]:
        """
        Iterate over all instance files.
        :return: Instance objects
        """
        for file_data in self._zipfile.filelist:
            if self._filename_fits_instance_convention(
                file_data.filename
            ) and not self._is_hidden_folder(file_data.filename):
                instance_name = self._extract_instance_name_from_path(
                    Path(file_data.filename)
                )
                try:
                    yield self._cache[instance_name]
                except KeyError:
                    yield self._cache_and_return(
                        self.read(self._zipfile.open(file_data.filename))
                    )

    def __getitem__(self, name: str) -> CGSHOP2026Instance:
        """
        Returns the instance of a specific name or throws an KeyError.
        :param name: Name of the instance.
        :return:
        """
        try:
            return self._cache[name]
        except KeyError:
            path = self._find_path(name)
            return self._cache_and_return(self.read(self._zipfile.open(path)))
