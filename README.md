The codes in this repo were used to generate the results for our paper (add arxiv link of the paper)

### Prerequisites

* MATLAB R2017a
* CPLEX (12.7.1), GUROBI (7.5.1), MOSEK (8.0.0), BARON (17), XPRESS (18.1)
* YALMIP R20171121 (<https://yalmip.github.io>)



### Folder Description

```
./mvpdp_small : solve mvpdp with 4-7 customers to optimality
./iqp_vs_milp : benchmark comparison of iqp against milp on solving svpdp with 8-50 customers within 3.5% optimality gap
./svpdp_manhattan : solve svpdp for real-world roadmap (openstreetmaps.org) and demands (www.nyc.gov) 
```
where refer to mvpdp as the multi-vehicle pickup and delivery problem, svpdp as the single-vehicle pickup and delivery problem, to iqp as the integer quadratic programming, to milp as the mixed integer linear programming

See README.md in each folder to find more detail on its contents

### Authors

* **Hyongju Park** 
* **Jinsun Liu**


### License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
