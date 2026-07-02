# LeMansAlgorithm

## Description
This is a discrete-event simulation for the pit crew of a Hypercar team during the 24 hours of Le Mans. It involves a comparison between 3 main strategies on how to allocate a single part (gearbox) for the three cars a team runs at one time. The first strategy tested is a FIFO algorithm that allocates the part to the first car that receives a part failure and needs replacement. The second strategy is a greedy algorithm which gives the part to the car with the highest value (higher position, more distance covered during the race). So if two cars have a failure at the same time, the higher value car gets the part. The third strategy is my new algorithm, a threshold based algorithm. It works similar to the greedy strategy by giving the part to a higher value car, but it holds the part when the urgency level of the request is lower. It holds the part in the hopes that a higher value car needs that part eventually instead. So if it is earlier in the race, the request is less urgent and there is more holding time for the part. However, the hold threshold decays over the length of the race. So, towards the end of the race, if there's a car that needs a part, the algorithm will allocate the part much quicker. Following the simulation, I compare it with an offline optimal benchmark that has full hindsight of the race to make the most optimal decision. This lets me see the efficiency of the algorithm, and which strategies make most sense.

## Requirements
- C++17 compiler
- CMake

## Build
```
cmake -S . -B build
cmake --build build
```
## Run
```
./build/LeMansAlgorithm
```
## Output
Running the program prints, in order:
- Three test cases, each showing expected vs. actual results (seed 12 no-contention, seed 28 contention, and the offline lower-bound check).
- Runtime and peak memory measurements for the full experiment.
- Aggregate results over 10,000 randomized races (average, min, and max weighted time lost per strategy).
- Competitive ratios of each strategy against the offline optimal benchmark.

## Reproducibility
- All randomness is seeded, so the program produces identical results on any machine. Each car's failures are generated from a fixed seed, and the offline benchmark regenerates the same failures before solving, so every run is fully reproducible.
