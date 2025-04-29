import commands2


class RobotContainer:
    def set_teleop_bindings() -> None: ...

    def get_auto(self) -> commands2.Command:
        return commands2.cmd.none()
