Add repo OSRF:

```
sudo apt update
sudo apt -y install wget lsb-release gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
```

install package libqz-cmake3-dev:
```
sudo apt install libgz-cmake3-dev
```

Add gz-plugin2:
```
sudo apt update
sudo apt install -y wget
wget -qO - https://packages.osrfoundation.org/gz.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] https://packages.osrfoundation.org/gz/ubuntu jammy main" > /etc/apt/sources.list.d/gz-latest.list'
sudo apt update
```

install gz-plugin2:
```
sudo apt install libgz-plugin2-dev
```

add gz-common5 repo:
```sudo apt update
sudo apt install -y wget
wget -qO - https://packages.osrfoundation.org/gz.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] https://packages.osrfoundation.org/gz/ubuntu jammy main" > /etc/apt/sources.list.d/gz-latest.list'
sudo apt update
```

install gz-common5:
```
sudo apt install libgz-common5-dev
```
