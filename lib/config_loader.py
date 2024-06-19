import tomli


class DotDict(dict):
    """
    A wrapper around the dict class that allows value access using "." notation.
    """

    def __getattr__(self, attr):
        try:
            value = self[attr]
            if isinstance(value, dict):
                return DotDict(value)
            return value
        except KeyError:
            raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{attr}'")

    def __setattr__(self, attr, value):
        if isinstance(value, dict):
            value = DotDict(value)
        self[attr] = value


class ConfigLoader:
    """
    Easy access to config values.

    Load values from files or manually with key value pairs. Each method also overwrites values if they already exist.
    """

    def load_file(self, filepath: str):
        """
        Initialize a ConfigLoader from a toml file.

        Args:
            filepath (str): Path to a toml file from the root of the project.

        Returns:
            self (ConfigLoader): Acts as an initialization method.
        """
        self.data = None

        with open(filepath, "rb") as toml_file:
            self.data: DotDict = (
                DotDict({**self.data, **tomli.load(toml_file)})
                if self.data
                else DotDict(tomli.load(toml_file))
            )

        return self

    def load_manual(self, **kwargs):
        """
        Initialize a ConfigLoader from keyword arguments.

        Args:
            **kwargs: Configuration key value pairs.

        Returns:
            self (ConfigLoader): Acts as an initialization method.
        """
        self.data: DotDict = DotDict({**self.data, **kwargs}) if self.data else DotDict(kwargs)

        return self


if __name__ == "__main__":
    loader = ConfigLoader().load_file("config/serial.toml")
    print(loader.data)