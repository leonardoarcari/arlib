# k-Shortest Paths with limited Overlap

The code in this repository was used in the following publications (please cite):

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
Exact and Approximate Algorithms for Finding k-Shortest Paths with Limited Overlap ,
In Proc. of the 20th Int. Conf. on Extending Database Technology (EDBT) (2017) 

- Theodoros Chondrogiannis, Panagiotis Bouros, Johann Gamper and Ulf Leser,
Alternative Routing: k-Shortest Paths with Limited Overlap ,
In Proc. of the 23rd ACM SIGSPATIAL Int. Conf. on Advances in Geographic Information Systems (GIS) (2015)

## Algorithms

The following table illustrates the inplemented algorithms:

| Abbreviation | Algorithm |
| ------ | ------ |
| op | The OnePass algorithm |
| mp | The MultiPass algorithm |
| opplus | The OnePass+ heuristic algorithm |
| svp | The SVP+ heuristic algorithm |
| esx | The ESX heuristic algorithm |

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

| Command | Description | Values |
| --- | --- | --- |
| PATHS | The number of requested result paths k | [0,+inf] |
| THRESHOLD | Similarity threshold θ | [0,1] |
| SRC | The source query node | [0,NUM_NODES] |
| TRG | The target query node | [0,NUM_NODES] |
| ALGORITHM | The selected algorithm | op\|mp\|opplus\|svpplus\|esx |

## License

This work is licensed under MIT License.
