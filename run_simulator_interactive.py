from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.code_editor.vscode")

print("Simulator is ready. You can now run code from VS Code.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()