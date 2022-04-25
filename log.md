# Installation

```
sudo apt-get install python-pip
sudo pip install -U rosdep

cd a1_patrol
sudo rosdep init
rosdep update
```

Unitree_legged_sdk Setup

```
mkdir -p /home/unitree/Unitree/sdk
cd /home/unitree/Unitree/sdk
clone legged_sdk & build
```



```
remote: Invalid username or password.

git remote remove origin
git remote add origin https://kimsooyoung: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@github.com/kimsooyoung/a1_patrol.git/

git push --set-upstream origin main
```

시작하기

시작하기 전에 랜선 뽑고 부팅

부팅 => L2 + A => L2 + Start => L1 + Start 제어


부팅 => (L1 + start)

L2 + A 하고 나서 L2 + Start로 ROS 제어 진입할 수 있는데 이때 조이스틱 왼쪽이 중립을 잘 유지해야 바뀐다.
