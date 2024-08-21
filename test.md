```shell
[INFO]  07-26-2024 15:43:56   test.c:83   (tid=131974226904896): mycase:
[INFO]  07-26-2024 15:43:56   test.c:94   (tid=131974226904896): seek from wrap 5, lpos 1000 to wrap 4, lpos 100
[DEBUG] 07-26-2024 15:43:56   test.c:97   (tid=131974226904896): start.wrap: 5, start.lpos: 1000, start.status: 0
[DEBUG] 07-26-2024 15:43:56   test.c:98   (tid=131974226904896): end.wrap: 4, end.lpos: 100, end.status: 1
[DEBUG] 07-26-2024 15:43:56   test.c:102  (tid=131974226904896): seekT: 9070 ms, beltW: 1101 lpos, motorW: 2 lpos
[INFO]  07-26-2024 15:43:56   test.c:110  (tid=131974226904896): read from wrap 4, lpos 100 to wrap 4, lpos 150
[DEBUG] 07-26-2024 15:43:56   test.c:114  (tid=131974226904896): beltW: 51 lpos, motorW: 0 lpos
[DEBUG] 07-26-2024 15:43:56   test.c:116  (tid=131974226904896): rwT: 10 ms
```

初始状态5-1000，寻址到4-100

从初始状态寻址到第一个io的起始位置，花了9070ms，是状态转换花费了时间吗
带体磨损为1101
电机磨损为2，与一次掉头操作相匹配

相比于寻址，读取操作从4-100读到4-150，只花了10ms

```shell
[INFO]  07-26-2024 15:43:56   test.c:94   (tid=131974226904896): seek from wrap 4, lpos 150 to wrap 125, lpos 58
[DEBUG] 07-26-2024 15:43:56   test.c:97   (tid=131974226904896): start.wrap: 4, start.lpos: 150, start.status: 1
[DEBUG] 07-26-2024 15:43:56   test.c:98   (tid=131974226904896): end.wrap: 125, end.lpos: 58, end.status: 1
[DEBUG] 07-26-2024 15:43:56   test.c:102  (tid=131974226904896): seekT: 5882 ms, beltW: 12593 lpos, motorW: 2 lpos
[INFO]  07-26-2024 15:43:56   test.c:110  (tid=131974226904896): read from wrap 125, lpos 58 to wrap 125, lpos 5
[DEBUG] 07-26-2024 15:43:56   test.c:114  (tid=131974226904896): beltW: 54 lpos, motorW: 0 lpos
[DEBUG] 07-26-2024 15:43:56   test.c:116  (tid=131974226904896): rwT: 10 ms
```

从4-150寻址到125-58
寻址时间5882ms
带体磨损12593，
电机磨损2，说明只掉了一次头

从io可以看出4的方向是顺着的
150->endpos
而125的方向是反着的
endpos->58

只掉了一次头
wrap间寻址造成的带体磨损：

```shell
[INFO]  07-26-2024 15:43:56   test.c:94   (tid=131974226904896): seek from wrap 125, lpos 5 to wrap 9, lpos 90
[DEBUG] 07-26-2024 15:43:56   test.c:97   (tid=131974226904896): start.wrap: 125, start.lpos: 5, start.status: 1
[DEBUG] 07-26-2024 15:43:56   test.c:98   (tid=131974226904896): end.wrap: 9, end.lpos: 90, end.status: 1
[DEBUG] 07-26-2024 15:43:56   test.c:102  (tid=131974226904896): seekT: 11365 ms, beltW: 12596 lpos, motorW: 4 lpos
[INFO]  07-26-2024 15:43:56   test.c:110  (tid=131974226904896): read from wrap 9, lpos 90 to wrap 9, lpos 30
[DEBUG] 07-26-2024 15:43:56   test.c:114  (tid=131974226904896): beltW: 61 lpos, motorW: 0 lpos
[DEBUG] 07-26-2024 15:43:56   test.c:116  (tid=131974226904896): rwT: 10 ms
```

125-5寻址到9-90

寻址时间：11365ms
带体磨损：12596ms
电机磨损：4

---

### 1

#### 同一wrap上的移动耗时
同一wrap上的寻址和读取耗时相同，每100单位耗时20ms

#### 不同wrap的相同位置间寻址造成的磨损

从奇数到偶数wrap，且相同位置，寻址一次：带体磨损3，电机磨损2

从奇数到奇数wrap，且相同位置，寻址一次：带体磨损12503，电机磨损4

从偶数到奇数wrap，且相同位置，寻址一次：带体磨损12501，电机磨损2

从偶数到偶数wrap，且相同位置，寻址一次：带体磨损1,电机磨损0

不同位置时，只需加上同一wrap不同位置间移动的带体磨损

#### 不同wrap的相同位置间寻址造成的耗时


### 从静止状态到运动状态：

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 100 1
请输入end.wrap, end.lpos, end.status: 1 0 1
seekT: 20 ms, beltW: 101 lpos, motorW: 0 lpos
rwT: 20 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 100 0
请输入end.wrap, end.lpos, end.status: 1 0 1
seekT: 6007 ms, beltW: 101 lpos, motorW: 0 lpos
rwT: 20 ms

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 1000 1
请输入end.wrap, end.lpos, end.status: 1 0 1
seekT: 197 ms, beltW: 1001 lpos, motorW: 0 lpos
rwT: 200 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 1000 0
请输入end.wrap, end.lpos, end.status: 1 0 1
seekT: 5740 ms, beltW: 1001 lpos, motorW: 0 lpos
rwT: 200 ms

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 500 0
请输入end.wrap, end.lpos, end.status: 1 0 1
seekT: 5891 ms, beltW: 501 lpos, motorW: 0 lpos
rwT: 100 ms

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 2 0
请输入end.wrap, end.lpos, end.status: 1 1 1
seekT: 6035 ms, beltW: 2 lpos, motorW: 0 lpos
rwT: 0 ms

---

### wrap间移动

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 2 0 1
seekT: 5503 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 3 0 1
seekT: 11006 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 100 1
请输入end.wrap, end.lpos, end.status: 2 100 1
seekT: 5503 ms, beltW: 201 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 100 1
请输入end.wrap, end.lpos, end.status: 3 100 1
seekT: 11006 ms, beltW: 12701 lpos, motorW: 4 lpos
rwT: 0 ms

---

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 101 0 1
seekT: 11300 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 100 0 1
seekT: 5797 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 201 0 1
seekT: 11600 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 200 0 1
seekT: 6097 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms

---

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 100 100 1
seekT: 5817 ms, beltW: 103 lpos, motorW: 2 lpos
rwT: 20 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 101 100 1
seekT: 11320 ms, beltW: 12603 lpos, motorW: 4 lpos
rwT: 20 ms

---

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 99 0 1
请输入end.wrap, end.lpos, end.status: 2 0 1
seekT: 5791 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 99 0 1
请输入end.wrap, end.lpos, end.status: 1 0 1
seekT: 11294 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 99 0 1
请输入end.wrap, end.lpos, end.status: 2 300 1
seekT: 5851 ms, beltW: 303 lpos, motorW: 2 lpos
rwT: 60 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 99 0 1
请输入end.wrap, end.lpos, end.status: 1 300 1
seekT: 11354 ms, beltW: 12803 lpos, motorW: 4 lpos
rwT: 60 ms

---

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 2 0 1
seekT: 5503 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 3 0 1
seekT: 11006 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 4 0 1
seekT: 5509 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 5 0 1
seekT: 11012 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 6 0 1
seekT: 5515 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 7 0 1
seekT: 11018 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 100 0 1
seekT: 5797 ms, beltW: 3 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 1 0 1
请输入end.wrap, end.lpos, end.status: 101 0 1
seekT: 11300 ms, beltW: 12503 lpos, motorW: 4 lpos
rwT: 0 ms

---

请输入start.wrap, start.lpos, start.status (或输入'q'退出): 2 0 1
请输入end.wrap, end.lpos, end.status: 3 0 1
seekT: 5503 ms, beltW: 12501 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 2 0 1
请输入end.wrap, end.lpos, end.status: 4 0 1
seekT: 11006 ms, beltW: 1 lpos, motorW: 0 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 2 0 1
请输入end.wrap, end.lpos, end.status: 5 0 1
seekT: 5509 ms, beltW: 12501 lpos, motorW: 2 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 2 0 1
请输入end.wrap, end.lpos, end.status: 6 0 1
seekT: 11012 ms, beltW: 1 lpos, motorW: 0 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 2 0 1
请输入end.wrap, end.lpos, end.status: 100 0 1
seekT: 11294 ms, beltW: 1 lpos, motorW: 0 lpos
rwT: 0 ms
请输入start.wrap, start.lpos, start.status (或输入'q'退出): 2 0 1
请输入end.wrap, end.lpos, end.status: 101 0 1
seekT: 5797 ms, beltW: 12501 lpos, motorW: 2 lpos
rwT: 0 ms