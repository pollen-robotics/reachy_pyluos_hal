from .register import Register


class ForceSensor:
    def __init__(self, id: int) -> None:
        self.id = id
        self.force = Register()

    def update_force(self, force: float):
        self.force.update(force)

    def get_force(self) -> float:
        return self.force.get()
