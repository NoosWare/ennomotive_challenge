/**
 * non-deterministic fuzzy gridworld.
 * Robot has crap IMU and even worse Odometry
 * So we can only approximate position with horrible accuracy.
 */
#include <iostream>
#include <unordered_set>
#include <random>
#include <ctime>
#include <chrono>

#include "relearn/src/relearn.hpp"

/// TODO: randomly explore, using some kind of a heuristic fitness function
template <typename S, typename A>
relearn::markov_chain<S,A> explore(
                                    const world & w,
                                    std::mt19937 & gen
                                  )
{
    using state = relearn::state<grid>;
    using action = relearn::action<direction>;

    std::uniform_int_distribution<unsigned int> dist(0, 3);

    bool stop = false;
    relearn::markov_chain<S,A> episode;

    // S_t (state now) is initially the root state
    grid curr  = w.start;
    auto state_now = state(curr.R, curr);

    // explore while Reward isn't positive or negative
    // and keep populating the episode with states and actions
    while (!stop) 
    {
        // randomly decide on next grid - we map numbers to a direction
        // and at the same time infer the next state
        unsigned int d = dist(gen);
        switch (d) {
            case 0 : curr.y--;
                     break;
            case 1 : curr.x++;
                     break;
            case 2 : curr.y++;
                     break;
            case 3 : curr.x--;
                     break;
        }
        // find the reward at the current coordinates
        auto it = w.blocks.find(curr);
        if (it != w.blocks.end()) {
            curr = *it;
            std::cout << "coord: " << curr.x << "," 
                      << curr.y << " = " << curr.R << std::endl;
            // create the action using direction as trait
            auto action_now = action(direction({d}));
            // add the state to the episode
            episode.emplace_back(relearn::link<state,action>{
                                  std::make_shared<state>(state_now),
                                  std::make_shared<action>(action_now)});
            // update current state to next state
            state_now = state(it->R, *it);
            if (it->R == -1 || it->R == 1) {
                stop = true;
            }
        }
        else {
            throw std::runtime_error("illegal block");
        }
    }
    // Add the terminal state last
    episode.emplace_back(relearn::link<state,action>{
                                  std::make_shared<state>(state_now),
                                  nullptr});
    return episode;
}

/// TODO: stay on policy: e.g., avoid collisions, obey traffic and don't flip over!!!
template <typename S, typename A>
void on_policy(const world & w, relearn::policy<S,A> & policy_map)
{
    grid curr = w.start;
    std::cout << "starting from: " << curr.x << "," 
              << curr.y << " = " << curr.R << std::endl;
    auto state_t = S(curr.R, curr);
    for (;;) {
        // get the best policy for this state from the episode
        if (auto action = policy_map.best_action(state_t)) {
            
            // how to infer the next state
            switch (action->trait().dir) {
                case 0 : curr.y--;
                         break;
                case 1 : curr.x++;
                         break;
                case 2 : curr.y++;
                         break;
                case 3 : curr.x--;
                         break;
            }
            std::cout << "best action: " << action->trait().dir << std::endl;
            auto it = w.blocks.find(curr);
            if (it != w.blocks.end()) {
                curr = *it;
                // calculate our next state
                auto state_n = S(curr.R, curr);
                std::cout << "coord: " << curr.x << "," 
                          << curr.y << " = " << curr.R << std::endl;
                state_t = state_n;
                if (curr.R == -1.0 || curr.R == 1.0) {
                    break;
                }
            }
            else {
                throw std::runtime_error("unknown grid");
            }       
        }
        else {
            throw std::runtime_error("no best action");
        }
    }
}

/// TODO: main entry point:
//        search for a QR-code until it is found and in front of us
//        else explore the environment, avoiding collisions
int main()
{
    std::mt19937 gen(static_cast<std::size_t>(
        std::chrono::high_resolution_clock::now().time_since_epoch().count()));

    // set shortcuts to state trait and action trait
    using state = relearn::state<grid>;
    using action = relearn::action<direction>;

    // create the world and populate it randomly
    world w = populate();
    relearn::policy<state,action> policies;
    std::vector<relearn::markov_chain<state,action>> episodes;

    for (;;) {
        // explore the grid world randomly - this produces an episode
        auto episode = explore<state,action>(w, gen);
        episodes.push_back(episode);

        // check solution has been found
        auto it = std::find_if(episode.begin(), episode.end(),
                 [&](const auto & link) {
                    return link.state_t->reward() == 1;
                 });
        if (it != episode.end()) {
            break;
        }
    }

    // use Q-learning algorithm to update the episode's policies
    auto learner = relearn::q_learning<state,action>(0.9, 0.1);
    for (int k = 0; k < 10; k++) {
        for (auto episode : episodes) {
            learner(episode, policies);
        }
    }

    // run on-policy and follow maxQ
    on_policy(w, policies);

    return 0;
}
