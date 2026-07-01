#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <cassert>
#include <chrono>
#include <algorithm>
#include <limits>

//Determine if the event is a part failure or the repair was complete
enum EventType {
    FAILURE, REPAIR_COMPLETE, RECONSIDER
};

//Structure for event of a car's part failing and needing replacement
struct Event {
    double time;
    int carID;
    EventType type;
};

//Tells the priority queue how to order the events
struct CompareEvent {
    bool operator()(const Event& a, const Event& b) {
        /* Flip the comparison to make the priority queue act as a min heap to
         * pop the smallest time first
         */
        return a.time > b.time;
    }
};

//Method to randomly generate failures per car
void createFailure(std::priority_queue<Event, std::vector<Event>, CompareEvent>& events,
                       int carID, double rate, double raceDuration,
                       std::mt19937& rng) {
    std::exponential_distribution<double> gap(rate);
    double time = 0.0;
    while (true) {
        time += gap(rng); //Jump the time forward by a randomly generated gap
        //Stop generating once the race has ended
        if (time >= raceDuration) {
            break;
        }
        events.push({time, carID, FAILURE});
    }
}

/* Method to calculate the current cost of a waiting request depending on how long it
 * has been in the race. Earlier failures have higher urgency to be replaced (high
 * value returned). Same failures later in race are lower urgency (lower value returned)
 */
double urgencyValue (double currentTime, double raceDuration, double weight) {
    return weight * (raceDuration - currentTime);
}

//Here will be the two strategies to be compared to my threshold algorithm strategy

/* First, we have a FIFO strategy. This is the strategy most commonly used by engineers
 * in real life. The first car to call in a failure gets the part.
 */
int FIFO_strategy(const std::vector<Event>& carWaitlist) {
    if (carWaitlist.empty()) {
        return -1;
    }
    return 0;
}

/* Next, we have the greedy strategy. The difference is this strategy picks the car with
 * the highest value waiting request. However, it never holds a spare part
 */
int greedy_strategy(const std::vector<Event>& carWaitlist, double currentTime,
                   double raceTime, const double carWeight[]) {
    if (carWaitlist.empty()) {
        return -1;
    }

    int bestRequest = 0;
    double bestVal = urgencyValue(currentTime, raceTime, carWeight[carWaitlist[0].carID]);

    //Search through the waiting requests
    for (int i = 1; i < (int)carWaitlist.size(); i++) {
        double temp = urgencyValue(currentTime, raceTime, carWeight[carWaitlist[i].carID]);
        //Compare to keep the best value and index updated throughout the time of the race
        if (temp > bestVal) {
            bestVal = temp;
            bestRequest = i;
        }
    }

    return bestRequest;
}

//My new algorithm: Threshold algorithm
/* Similar to the greedy strategy, but holds the spare parts in reserve
 * for better outcomes on average. It only allocates a spare part if the
 * urgency value crosses a certain threshold.
 */
int threshold_strategy(const std::vector<Event>& carWaitlist, double currentTime,
                      double raceTime, const double carWeight[], double waitThreshold) {
    if (carWaitlist.empty()) {
        return -1;
    }

    double bestVal = urgencyValue(currentTime, raceTime, carWeight[carWaitlist[0].carID]);

    //Same as the greedy strategy here, search through the waiting requests
    int bestRequest = 0;
    for (int i = 1; i < (int)carWaitlist.size(); i++) {
        double temp = urgencyValue(currentTime, raceTime, carWeight[carWaitlist[i].carID]);
        //Compare to keep the best value and index updated throughout the time of the race
        if (temp > bestVal) {
            bestVal = temp;
            bestRequest = i;
        }
    }

    //Compare car weight against threshold value
    /* Higher car weights get cross the threshold before lower weights
     * do. This leaves the mechanics free for the higher priority
     * repair requests
     */
    double waitCost = carWeight[carWaitlist[bestRequest].carID] *
                      (currentTime - carWaitlist[bestRequest].time);

    if (carWeight[carWaitlist[bestRequest].carID] < 0.9 && waitCost < waitThreshold) {
        return -1; //low priority car hasn't waited long enough, hold part
    }

    return bestRequest;
}

/* Offline strategy to look into the future of a race (full hindsight) and allocate
 * the parts for the most optimal time. Finds the most optimal decisions in the race
 * to compare with how the other strategies did without knowing the future. Claude
 * helped me design this function.
 */
double offlineOptimalLoss(int numCars, double raceTime, double failRate,
                          double repairDuration, const double carWeight[], int baseSeed) {
    //Regenerate the exact same failure sequence used by runSimulation
    std::vector<Event> failures;
    for (int carID = 0; carID < numCars; carID++) {
        std::mt19937 rng(baseSeed + carID);
        std::exponential_distribution<double> gap(failRate);
        double t = 0.0;
        while (true) {
            t += gap(rng);
            if (t >= raceTime) break;
            failures.push_back({t, carID, FAILURE});
        }
    }
    int n = (int)failures.size();
    if (n == 0) return 0.0;
    std::sort(failures.begin(), failures.end(),
              [](const Event& a, const Event& b) { return a.time < b.time; });

    const double DOMINANCE_EPS = 1e-12; //tolerance when comparing (freeTime, cost) states

    //dp[mask] = non-dominated (machineFreeTime, cost) states after servicing 'mask'
    std::vector<std::vector<std::pair<double, double>>> dp(1 << n);
    dp[0].push_back({0.0, 0.0});
    double best = std::numeric_limits<double>::max();

    for (int mask = 0; mask < (1 << n); mask++) {
        if (dp[mask].empty()) continue;
        if (mask == (1 << n) - 1) {
            for (auto& state : dp[mask]) best = std::min(best, state.second);
            continue;
        }
        for (auto& state : dp[mask]) {
            double freeTime = state.first;
            double cost     = state.second;
            for (int k = 0; k < n; k++) {
                if (mask & (1 << k)) continue; //already serviced
                double release = failures[k].time;
                double weight  = carWeight[failures[k].carID];
                double start   = std::max(freeTime, release); //may idle to wait for this car
                double newCost = cost + weight * (start - release);
                double newFree = start + repairDuration;
                int nextMask   = mask | (1 << k);

                //Insert the new state, keeping only the Pareto frontier
                auto& states = dp[nextMask];
                bool dominated = false;
                for (auto& q : states)
                    if (q.first <= newFree + DOMINANCE_EPS && q.second <= newCost + DOMINANCE_EPS) {
                        dominated = true;
                        break;
                    }
                if (dominated) continue;
                states.erase(std::remove_if(states.begin(), states.end(),
                    [&](std::pair<double, double>& q) {
                        return q.first >= newFree - DOMINANCE_EPS && q.second >= newCost - DOMINANCE_EPS;
                    }), states.end());
                states.push_back({newFree, newCost});
            }
        }
    }
    return best;
}

enum Strategy {FIFO, GREEDY, THRESHOLD};

//Method to run full 24 hour race simulation based on specified strategy
//Calculates the weighted time lost. The lower the loss the better
double runSimulation(Strategy strategy, int numCars, double raceTime,
                     double failRate, double repairDuration, const double carWeight[], int baseSeed) {
    //Initialize the priority queue of events
    std::priority_queue<Event, std::vector<Event>, CompareEvent> events;

    //Track the cars that have had a failure but haven't received a replacement part
    std::vector<Event> carWaitlist;

    bool reconsidering = false; //tracks if a RECONSIDER is already scheduled
    bool partAvailable = true;
    double totalLost = 0.0;
    const double MAX_HOLD = 0.5; //Maximum time a part can be held before forced allocation

    //Create the failures per car using the base seed and car ID
    for (int carID = 0; carID < numCars; carID++) {
        std::mt19937 rng(baseSeed + carID);
        createFailure(events, carID, failRate, raceTime, rng);
    }

    //Pop events one at a time to process each failure
    while (!events.empty()) {
        Event e = events.top(); //Peek at the smallest time event
        events.pop();

        //Computation on the wait threshold for the threshold strategy
        /* Basically, this is the number that decides how long can we
         * afford to wait for a better car to arrive in the pit
         * before we need to allocate the part. Threshold value naturally
         * decays as the race approaches the finish. Threshold multiplier
         * is used to ensure cars have to be in a higher position so the
         * car in the last place for the team (weight 0.8) is filtered out to
         * prioritize higher priority requests
         */
        double waitThreshold = MAX_HOLD * (1.0 - e.time / raceTime);

        //If repair just finished, free up the mechanics
        if (e.type == REPAIR_COMPLETE) {
            partAvailable = true;
        }
        else if (e.type == RECONSIDER) {
            reconsidering = false;
        }
        else {
            //Add car to the waitlist for new part failure
            carWaitlist.push_back(e);
        }

        //If the part is available and the mechanics are not currently repairing a car
        if (partAvailable && !carWaitlist.empty()) {
            int allocated = -1;

            //Allocate the part based on the rules of the given strategy
            if (strategy == FIFO) {
                allocated = FIFO_strategy(carWaitlist);
            }
            else if (strategy == GREEDY) {
                allocated = greedy_strategy(carWaitlist, e.time, raceTime, carWeight);
            }
            else if (strategy == THRESHOLD) {
                allocated = threshold_strategy(carWaitlist, e.time, raceTime, carWeight, waitThreshold);
            }

            //If part is given to the car, mark the part as no longer available and remove the request
            if (allocated != -1) {
                partAvailable = false;
                //Set the repair duration to 1 hour until completed and mechanics are freed again
                double waitTime = e.time - carWaitlist[allocated].time;
                totalLost += carWeight[carWaitlist[allocated].carID] * waitTime;
                events.push({e.time + repairDuration, carWaitlist[allocated].carID, REPAIR_COMPLETE});
                carWaitlist.erase(carWaitlist.begin() + allocated);
            }
            else if (strategy == THRESHOLD && !carWaitlist.empty()) {
                //schedule forced re-evaluation at exactly the moment the best waiting car's hold period expires
                if (!reconsidering) {
                    int bestIndex = 0;
                    double bestW = carWeight[carWaitlist[0].carID];
                    for (int i = 1; i < (int)carWaitlist.size(); i++) {
                        if (carWeight[carWaitlist[i].carID] > bestW) {
                            bestW = carWeight[carWaitlist[i].carID];
                            bestIndex = i;
                        }
                    }
                    double expireTime = carWaitlist[bestIndex].time + (waitThreshold / carWeight[carWaitlist[bestIndex].carID]) + 1e-9;
                    if (expireTime < raceTime) {
                        events.push({expireTime, -1, RECONSIDER});
                        reconsidering = true; //Prevents duplicate reconsider events
                    }
                }
            }
        }
    }

    //If a car is still on the waitlist and never got the part, mark time lost
    for (int i = 0; i < (int)carWaitlist.size(); i++) {
        double waitTime = raceTime - carWaitlist[i].time;
        totalLost += carWeight[carWaitlist[i].carID] * waitTime;
    }

    return totalLost;
}

int main() {
    //Initialize the number of cars on the grid for the team
    const int NUM_CARS = 3;
    //Initialize race time (24 hours of Le Mans)
    const double RACE_TIME = 24.0;

    //Add prioritization weights to the cars based on which one is in front
    const double CAR_WEIGHT[3] = {1.0, 1.2, 0.8};

    //Create the failure rate for the cars
    /* Uniform rate for all cars for testing purposes, however later implementation would include
    * unique rates for each car. The rate was based on statistical averages for what an actual
    * car racing in Le Mans would experience (using Claude)
    */
    const double FAIL_RATE = 0.1; //Failures per hour

    //Starting seed
    /* I chose 12 because that is the number for one of the cars on Cadillac Hertz Team Jota,
     * who I was rooting to win the race this year (they held 1st position for half the race but
     * ended up finishing 4th :(
     * 12 does end up yielding 0's on the output but that is expected for some seeds and works fine.
     */
    const int SEED = 12;

    //Realistic time for a gearbox replacement during the Le Mans race
    /* I chose for this simulation to just focus on the gearbox component
     * to replace on the cars. More components can be added. In a Le Mans
     * race, a realistic time for a gearbox replacement would be about 1
     * hour
     */
    const double REPAIR_DURATION = 1.0;

    //Non-trivial test case 1: Single race using seed 12
    double FIFO_lost = runSimulation(FIFO,      NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, SEED);
    double greedy_lost = runSimulation(GREEDY,      NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, SEED);
    double threshold_lost = runSimulation(THRESHOLD,      NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, SEED);
    double offline_lost   = offlineOptimalLoss(NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, SEED);

    std::cout << "TEST 1 (seed 12, no contention):\n";
    std::cout << "  Expected: FIFO 0, Greedy 0, Threshold 0, Offline 0\n";
    std::cout << "  Actual:   FIFO " << FIFO_lost << ", Greedy " << greedy_lost
              << ", Threshold " << threshold_lost << ", Offline " << offline_lost << "\n";

    //Non-trivial test case 2: Contention race (seed 28), known expected values
    double f28 = runSimulation(FIFO,      NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, 28);
    double g28 = runSimulation(GREEDY,    NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, 28);
    double t28 = runSimulation(THRESHOLD, NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, 28);
    double o28 = offlineOptimalLoss(NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, 28);

    std::cout << "\nTEST 2 (seed 28, contention):\n";
    std::cout << "  Expected: FIFO 2.9062, Greedy/Threshold/Offline 2.7062\n";
    std::cout << "  Actual:   FIFO " << f28 << ", Greedy " << g28
              << ", Threshold " << t28 << ", Offline " << o28 << "\n";

    //Non-trivial test case 3: Offline optimum is a lower bound on every strategy (seed 28)
    bool lowerBound = (o28 <= f28) && (o28 <= g28) && (o28 <= t28);
    std::cout << "\nTEST 3 (offline is a valid lower bound, seed 28):\n";
    std::cout << "  Expected: offline <= FIFO, Greedy, and Threshold  ->  true\n";
    std::cout << "  Actual:   " << (lowerBound ? "true" : "false") << "\n";

    //Offline simulation for best possible result in race
    std::cout << "Offline optimal time lost: " << offline_lost << std::endl;
    std::cout << std::endl;

    //Trial loop using 10,000 races with different seeds
    const int NUM_TRIALS = 10000;

    double FIFO_total = 0.0,   FIFO_min = 1e9,   FIFO_max = 0.0;
    double greedy_total = 0.0, greedy_min = 1e9, greedy_max = 0.0;
    double threshold_total = 0.0, threshold_min = 1e9, threshold_max = 0.0;
    double offline_total = 0.0, offline_min = 1e9, offline_max = 0.0;

    //Races where the threshold algorithm beats the FIFO and Greedy
    int thresholdWins = 0;

    //Run the 10,000 simulations
    for (int i = 0; i < NUM_TRIALS; i++) {
        //Each trial gets a unique seed starting from 12
        int seed = SEED + i;

        //Run the simulations
        double f = runSimulation(FIFO,      NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, seed);
        double g = runSimulation(GREEDY,    NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, seed);
        double t = runSimulation(THRESHOLD, NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, seed);
        double o   = offlineOptimalLoss(NUM_CARS, RACE_TIME, FAIL_RATE, REPAIR_DURATION, CAR_WEIGHT, seed);

        FIFO_total += f;   FIFO_min = std::min(FIFO_min, f);   FIFO_max = std::max(FIFO_max, f);
        greedy_total += g; greedy_min = std::min(greedy_min, g); greedy_max = std::max(greedy_max, g);
        threshold_total += t; threshold_min = std::min(threshold_min, t); threshold_max = std::max(threshold_max, t);
        offline_total += o; offline_min = std::min(offline_min, o); offline_max = std::max(offline_max, o);

        //Increment threshold wins if it beats the other two algorithms
        if (t < f && t < g) {
            thresholdWins++;
        }
    }

    /* Print the results, displaying the total time lost, minimum time lost
     * (best races), and maximum time lost (worst races) for each strategy over the 10,000 races
     */
    std::cout << "TRIAL RESULTS (10,000 races): " << std::endl;
    std::cout << "Strategy    Avg Loss    Min Loss    Max Loss\n";
    std::cout << "FIFO        " <<   FIFO_total/NUM_TRIALS   << "      " << FIFO_min   << "      " << FIFO_max   << "\n";
    std::cout << "Greedy      " <<  greedy_total/NUM_TRIALS << "      " << greedy_min << "      " << greedy_max << "\n";
    std::cout << "Threshold   " <<  threshold_total/NUM_TRIALS << "      " << threshold_min << "      " << threshold_max << "\n";
    std::cout << "Threshold won " << thresholdWins << "/" << NUM_TRIALS << " races (" << (100.0 * thresholdWins / NUM_TRIALS) << "%)\n";

    //Compare results with the offline optimal benchmark
    std::cout << "\n--- Competitive Ratios vs Offline Optimal ---\n";
    std::cout << "FIFO:      " << (FIFO_total / offline_total) << "\n";
    std::cout << "Greedy:    " << (greedy_total / offline_total) << "\n";
    std::cout << "Threshold: " << (threshold_total / offline_total) << "\n";
}

