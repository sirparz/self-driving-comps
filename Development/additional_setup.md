# Additional Setup
Congratulation, you have made it to the setup stage. Now it's time to add an executable terminal leading to its respective environment. This will save steps on going through the commands to enter the environment.
## Quanser Virtual Environment
Should there be any update to the virtual environment issued by Quanser, please pull the latest docker image using the command
```bash
docker pull quanser/acc2025-virtual-qcar2:latest
```
Create the the terminal file in the desktop.
```bash
nano ~/Desktop/quanser_virtual_env.desktop
```
Add the following content
```ini
[Desktop Entry]
Name=Quanser Virtual Environment Container
Comment=Launch Quanser Virtual QCar2 Container
Exec=gnome-terminal -- bash -c 'cd /home/$USER/Documents/ACC_Development/docker/virtual_qcar2; sudo docker run --rm -it --network host --name virtual-qcar2 quanser/acc2025-virtual-qcar2 bash; exec bash'
Icon=utilities-terminal
Type=Application
Categories=Development;TerminalEmulator;
```
**Save and exit** (`CTRL+X`, then `Y`, then `Enter`) and make it executable.
```bash
chmod +x ~/Desktop/quanser_virtual_env.desktop
```
**Enable Execution in GUI**:
-   Right-click **`quanser_virtual_env.desktop`** on your **Desktop**.
-   Select **"Allow Launching"**.
## Isaac ROS Container
### Bash Profile
The step used will be different from the previous section as a config needs to be set in the gnome terminal.
 1. First, open the terminal, click **≡** then go to **Preferences** 
 2. In the **Profile** section of the setting, add a new profile **+** and
    name it `Isaac ROS Container`.
 3. In the profile created, go to **Command** section in the top bar, set the **Preserve working directory** as `Always` and **When command exits** as `Exit the terminal`
 4. Check **Run a custom command instead of my shell**, and fill the **Custom command**:
  ```bash
bash -c 'echo -ne "\033]0;Isaac ROS Container\007"; cd /home/$USER/Documents/ACC_Development/isaac_ros_common; ./scripts/run_dev.sh /home/$USER/Documents/ACC_Development/Development; source ros2/install/setup.bash; exec bash;'
```
The profile is done, feel free to modify the **Text** and **Colors** section to your liking.

### Terminal File
Create the the terminal file in the desktop.
```bash
nano ~/Desktop/isaac_ros_container.desktop
```
Add the following content
```ini
[Desktop Entry]
Name=Isaac ROS Container
Comment=Launch Isaac ROS Development Terminal
Exec=gnome-terminal --window --profile="Isaac ROS Container"
Icon=utilities-terminal
Type=Application
Categories=Development;TerminalEmulator;
```
**Save and exit** (`CTRL+X`, then `Y`, then `Enter`) and make it executable.
```bash
chmod +x ~/Desktop/isaac_ros_container.desktop
```
**Enable Execution in GUI**:
-   Right-click **`isaac_ros_container.desktop`** on your **Desktop**.
-   Select **"Allow Launching"**.


