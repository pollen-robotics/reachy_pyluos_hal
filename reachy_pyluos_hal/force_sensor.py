from .register import Register


class ForceSensor:
    def __init__(self, id: int) -> None:
        self.id = id
        self.force = Register()

    def __repr__(self) -> str:
        """Represent force sensor."""
        return f'<ForceSensor id={self.id}>'

    def update_force(self, force: float):
        self.force.update(force)

    def get_force(self) -> float:
        return self.force.get()
