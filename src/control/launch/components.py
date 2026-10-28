from bringup.launching import python,owned
def components(run):
    commands=[('controller',python('control.controller','--run',run)),
        ('command_bridge',python('control.bridge','--runtime-config',run/'runtime_context.json','--command-file',run/'navigation_command.json')),
        ('mission',python('control.mission.mission_runtime','--run',run))]
    return [action for role,command in commands for action in owned(role,command,run)]
