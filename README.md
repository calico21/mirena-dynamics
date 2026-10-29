# Mirena Workspace
Welcome to the mirena project workspace, here you can start off with the development of 
the autopilot system for Tecnun-eRacing TeR Vehicle, you just have to clone the repo, and initialize
all its submodules, there you can build them and launch them using colcon.

# Getting started
As always, start by cloning the repository and its submodules:

```{bash}
git clone https://git.eracing.es/Driverless/mirena_ws/ --recurse-submodules
```
## DevContainer
The recommended way of developing mirena_sim is using the bundled devcontainer configuration,
download VSCode and install the devcontainer plugin. Use it to create a working dev environment.

If you have NVIDIA graphics installed dont forget to install 
[nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
set it as default runtime by editing `/etc/docker/daemon.json`

```{json}
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
```
```{bash}
sudo systemctl restart docker
```




## Building ros modules
Finally, you can build the full pipeline with:
```{bash}
cd mirena_ws # Also, make sure ros2's setup file is sourced.

colcon build 
source ./install/local_setup.zsh
```


> [!TIP]
> Si fuera fácil, lo haría cualquiera, hazlo.  
> **Ozuba.**
