# k-Shortest Paths with limited Overlap

The code in this repository was used in the following publications (please cite):

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited Overlap ,
In Proc. of the 20th Int. Conf. on Extending Database Technology (EDBT) (2017) 

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
Alternative Routing: k-Shortest Paths with Limited Overlap ,
In Proc. of the 23rd ACM SIGSPATIAL Int. Conf. on Advances in Geographic Information Systems (GIS) (2015)

## Tests

In order to run the program and test the algorithms you must first compile the source code. In order to compile, simply

```sh
$ make
```

Then, in order to run the run.exec with the sample file:

```sh
$ ./run.exec -f sample/sample.gr -k [PATHS] -s [THRESHOLD] -S [SRC] -T [TRG] -a [ALGORITHM]
```
The following table shows the possible values for each parameter.

| Parameter | Explanation | Values |
| ------ | ------ |
| PATHS | The number of requested result paths k | 0 |
| THRESHOLD | [test | 0 |
| SRC | The source query node | [0,\|N\|] |
| TRG | The target query node | [0,\|N\|] |
| ALGORITHM | The selected algorithm | op\|mp\|opplus\|svpplus\|esx |

| Command | Description |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |

## License

This work is licensed under MIT License.
