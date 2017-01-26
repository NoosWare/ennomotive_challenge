/**
 * @struct robot_state
 * @brief used to describe the current internal and external state of the robot
 *
 * @TODO: obtain this by subscribing to `/laser` and `ping`, and `traffic`
 **/
struct robot_state
{
    rplidar lazor;
    qr_info target;
    bool    traffic;
    unsigned int ping;

    double R;

    bool operator==(const grid & arg) const
    {
        return (this->x == arg.x) && 
               (this->y == arg.y);
    }
};

/**
 * @brief constructs `robot` states
 *        by subscribing to topics and services
 */
class state_factory
{
public:


private:
};

/**
 * @struct robot_action
 * @brief possible actions:
 *   - 0 for left
 *   - 1 for forward
 *   - 2 for right
 *   - 3 for down
 *   - 4 for wait/nothing/stop.
 *
 * @TODO: translate this to a motor command (e.g, publish message)
 **/
struct robot_action
{
    unsigned int dir;

    bool operator==(const direction & arg) const
    {
        return (this->dir == arg.dir);
    }

    /// @brief run this action (e.g., command motors)
    void operator()()
    {

    }
};

/**
 * Hash specialisations in the STD namespace for structs grid and direction.
 */
namespace std 
{
template <> struct hash<robot_state>
{
    std::size_t operator()(grid const& arg) const 
    {
        std::size_t seed = 0;
        relearn::hash_combine(seed, arg.x);
        relearn::hash_combine(seed, arg.y);
        return seed;
    }
};

template <> struct hash<robot_action>
{
    std::size_t operator()(direction const& arg) const
    {
        std::size_t seed = 0;
        relearn::hash_combine(seed, arg.dir);
        return seed;
    }
};
}

