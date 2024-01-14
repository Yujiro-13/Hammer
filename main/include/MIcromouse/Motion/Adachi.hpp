#ifndef ADACHI_HPP
#define ADACHI_HPP

#include "motion.hpp"

class Adachi : public Motion
{
public:
    void search_adachi(int gx, int gy);
    void InitMaze();

private:
    void init_map(int x, int y);
    void make_map(int x, int y, int mask);
    void set_wall(int x, int y);
    t_bool is_unknown(int x, int y);
    int get_priority(int x, int y, t_direction dir);
    int get_nextdir(int x, int y, int mask, t_direction *dir);
};

#endif // ADACHI_HPP