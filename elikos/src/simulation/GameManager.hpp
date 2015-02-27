#ifndef SIM_GAME_MANAGER_HPP
#define SIM_GAME_MANAGER_HPP


namespace elikos_sim {

class GameManager {
public:
    GameManager();
    ~GameManager();

private:
    int score;

    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    GameManager& operator= (const GameManager&);
    GameManager (const GameManager&);

};

} // namespace elikos_sim

#endif // SIM_GAME_MANAGER_HPP
