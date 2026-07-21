#### Install

step0
```
mkdir -p ~/colcon_ws
```

step1
```
cd ~/colcon_ws
git clone https://github.com/kyo0221/rsf_team_nav.git src
```

デプロイ時
```
vcs import src < src/deployment.repos
```

step2
```
colcon build --symlink-install
```
