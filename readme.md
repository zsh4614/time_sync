# time_sync
a time synchronizer based ros.
more information: [blog]()
# quick start
1.download project and build

```
git clone git@github.com:zsh4614/time_sync.git
cd time_sync
mkdir build && cd build
cmake ..
make
```
2.download demo data

link: [google drive](https://drive.google.com/u/0/uc?export=download&confirm=_ozV&id=1tLNqracjp6NG9YxHS2kg6ZJsaHe6jZc-)

3.run the demo

```
cd build
./time_sync_demo
```

4.start a roscore

```$xslt
roscore
```

5.run the demo data

```$xslt
rosbag play data.bag
```

then you will see:
![Screenshot from 2021-12-09 15-03-57.png](https://s2.loli.net/2021/12/09/un9ObjVghyLF6l2.png)