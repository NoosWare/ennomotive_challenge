#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

/**
 * @struct robot_state
 * @brief used to describe the current internal and external state of the robot
 * @TODO: obtain this by subscribing to `/laser` and `ping`, and `traffic`
 **/
struct robot_state
{
    double x;
    double y;

    //qr_info target;
    bool    traffic;
    unsigned int ping;

    double R;

    bool operator==(const grid & arg) const
    {
        return (this->x == arg.x) && 
               (this->y == arg.y);
    }
};

class 

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
}
#endif
