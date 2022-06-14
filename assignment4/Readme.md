## Assignment 4

### A star planning algorithm

장애물(collision)을 피해 현재 위치에서 타겟 위치로 이동하는 경로를 구하였다. 
우선 맵을 불러오거나 센서로 맵을 만드는 방법을 배우고, 이를 토대로 장애물 위치를 표시하는 방법을 배웠다. 그리고 graph-search 를 통한 motion planning 을 진행하였다. 

> Motion planning 이란 물체를 source 부터 target destination 까지의 경로를 plan 하는 과정이다. 

이번 과제에서는 A* 알고리즘을 사용하여 상하좌우 어디로 이동할지 planning을 하였다. A* 알고리즘은 graph-search 이므로 공간을 단위면적으로 쪼개서 grid 처럼 생각을 했고, 상하좌우로 1 칸씩 이동할 수 있게 했다. 이를 통해 주어진 A_star.cpp 파일의 내장 함수들을 사용하여 경로를 탐색하는 방법을 배웠다. 또한, 이번 과제에서는 장애물 주위에도 장애물이 존재할 수 있는 확률을 주어 보다 안전한 경로를 구상할 수 있게 했다.


이번 과제에서 사용한 A* 알고리즘 외에도 Rapidly exploring Random Tree(RRT) 알고리즘도 존재한다. 또한, graph-search 를 위한 motion planning 이외에도 sampling-based 혹은 incremental-search(RRT) method 도 존재한다. Sampling based motion planning 은 먼저 sample 을 만들어서 sample 마다 cost 를 구한 뒤 최소 cost 의 경로를 구하는 방법이다. Incremental-search based motion planning 은 grid 를 사용하지 않는 대신 경로에 collision 이 일어나지 않도록 구상하는 방법이다. 이 방법은 특이 3D 공간에서 유용하다.


### more about A* Algorithm
A* 알고리즘은 주어진 출발점에서 타겟 지점까지 가는 최단 경로를 찾는다. Graph-based 알고리즘이 그러듯 grid 가 필요하다. A*는 각 지점에 목표까지의 최단 경로를 추정하는 값인 heuristic 을 지정한다. 따라서 경로를 평가하는 함수인 f(n)은 경로 상의 한 지점까지의 거리(혹은 가중치)인 g(n)과 그 지점부터 목표까지의 추정 값 h(n)의 합으로 나타낼 수 있다. 따라서 가장 작은 f(n)값을 따라가다 보면 목표지점에 도달한다. 이 때 거리를 구하는 방법에는 세 가지가 있다. 출발점과 목표지점까지의 직선거리인 Euclidian, x 축상의 변위와 y 축상의 변위를 더한 Manhattan, 대각선의(diagonal) 거리도 계산한 Octagonal 이 있다. 이번 과제에서는 diagonal 한 움직임을 주지 않았기 때문에 본 코드에서는 Manhattan 방법을 사용했다. 본 과제에서 같은 코드로 heuristic 만 바꾸어 준 결과는 아래에 있다. 큰 차이는 발견하지 못했다.


  Eucledian

  Manhattan

  Octagonal




