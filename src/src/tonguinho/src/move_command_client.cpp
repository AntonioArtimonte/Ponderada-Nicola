#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <vector>
#include <queue>
#include <string>
#include <array>
#include <iostream>

using namespace std::chrono_literals;

struct Position {
    int x;  // Coluna
    int y;  // Linha

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

enum class RobotState {
    EXPLORING,
    MOVING_TO_TARGET
};

class MoveServiceClient : public rclcpp::Node
{
public:
    MoveServiceClient() 
        : Node("move_service_client"), 
          state_(RobotState::EXPLORING), 
          replan_attempts_(0),
          max_replan_attempts_(3)
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        grid_ = std::vector<std::vector<char>>(20, std::vector<char>(20, 'U')); 
        robot_pos_ = {11, 5};  
        target_pos_ = {-1, -1};  
        last_direction_ = "";

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service '/move_command'...");
        }

        grid_[robot_pos_.y][robot_pos_.x] = 'R';

        send_initial_move();
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    std::vector<std::vector<char>> grid_;
    Position robot_pos_;
    Position target_pos_;
    std::queue<std::string> planned_moves_;
    std::string last_direction_;
    RobotState state_;
    int replan_attempts_;
    const int max_replan_attempts_;

    void send_initial_move()
    {
        RCLCPP_INFO(this->get_logger(), "Iniciando exploracao...");
        plan_exploration_path();
    }

    void send_move_request(const std::string &direction)
    {
        last_direction_ = direction; 

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        RCLCPP_INFO(this->get_logger(), "Tentando mover %s da posicao (%d, %d)", direction.c_str(), robot_pos_.x, robot_pos_.y);

        auto future_result = client_->async_send_request(request,
            std::bind(&MoveServiceClient::process_response, this, std::placeholders::_1));
    }

    void process_response(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture response_future)
    {
        auto response = response_future.get();

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Movimento %s feito com sucesso", last_direction_.c_str());

            grid_[robot_pos_.y][robot_pos_.x] = 'F';

            robot_pos_.x = response->robot_pos[1]; 
            robot_pos_.y = response->robot_pos[0];
            grid_[robot_pos_.y][robot_pos_.x] = 'R';

            RCLCPP_INFO(this->get_logger(), "Robo moveu para posicao (%d, %d)", robot_pos_.x, robot_pos_.y);

            if (target_pos_.x == -1) {
                target_pos_.x = response->target_pos[1];
                target_pos_.y = response->target_pos[0];
                grid_[target_pos_.y][target_pos_.x] = 'T';
                RCLCPP_INFO(this->get_logger(), "Posicao do target identificada em (%d, %d)", target_pos_.x, target_pos_.y);
            }

            update_grid(response);

            if (state_ == RobotState::EXPLORING && is_exploration_complete()) {
                RCLCPP_INFO(this->get_logger(), "Exploracao completa. Planejando caminho para o target.");
                state_ = RobotState::MOVING_TO_TARGET;
                RCLCPP_INFO(this->get_logger(), "Estado atualizado para MOVING_TO_TARGET.");
                plan_path_to_target();
            } else if (state_ == RobotState::EXPLORING) {
                plan_exploration_path();
            } else if (state_ == RobotState::MOVING_TO_TARGET) {
                if (!planned_moves_.empty()) {
                    execute_planned_moves();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Target alcancado!");
                    print_grid();
                    rclcpp::shutdown();
                }
            }

        } else {
            RCLCPP_WARN(this->get_logger(), "Falha para mover %s da posicao (%d, %d)", last_direction_.c_str(), robot_pos_.x, robot_pos_.y);

            Position blocked_pos = get_neighbor_position(robot_pos_.x, robot_pos_.y, last_direction_);
            if (is_in_bounds(blocked_pos)) {
                grid_[blocked_pos.y][blocked_pos.x] = 'B';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) marcada como bloqueada", blocked_pos.x, blocked_pos.y);
            }

            planned_moves_ = std::queue<std::string>(); 
            RCLCPP_INFO(this->get_logger(), "Movimentos planejados limpos. Replanejando caminho...");

            if (state_ == RobotState::EXPLORING) {
                plan_exploration_path();
            } else if (state_ == RobotState::MOVING_TO_TARGET) {
                RCLCPP_INFO(this->get_logger(), "Falha ao mover durante MOVING_TO_TARGET. Replanejando caminho para o target.");
                plan_path_to_target();
            }
        }
    }

    void update_grid(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response> &response)
    {
        int x = robot_pos_.x;
        int y = robot_pos_.y;

        RCLCPP_INFO(this->get_logger(), "Atualizando a grid...");

        // Esquerda
        if (is_in_bounds({x - 1, y})) {
            if (response->left == "b") {
                grid_[y][x - 1] = 'B';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta bloqueada (esquerda)", x - 1, y);
            } else if (grid_[y][x - 1] == 'U') {
                grid_[y][x - 1] = 'F';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta livre (esquerda)", x - 1, y);
            }
        }

        // Direita
        if (is_in_bounds({x + 1, y})) {
            if (response->right == "b") {
                grid_[y][x + 1] = 'B';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta bloqueada (direita)", x + 1, y);
            } else if (grid_[y][x + 1] == 'U') {
                grid_[y][x + 1] = 'F';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta livre (direita)", x + 1, y);
            }
        }

        // Cima
        if (is_in_bounds({x, y - 1})) {
            if (response->up == "b") {
                grid_[y - 1][x] = 'B';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta bloqueada (cima)", x, y - 1);
            } else if (grid_[y - 1][x] == 'U') {
                grid_[y - 1][x] = 'F';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta livre (cima)", x, y - 1);
            }
        }

        // Baixo
        if (is_in_bounds({x, y + 1})) {
            if (response->down == "b") {
                grid_[y + 1][x] = 'B';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta bloqueada (baixo)", x, y + 1);
            } else if (grid_[y + 1][x] == 'U') {
                grid_[y + 1][x] = 'F';
                RCLCPP_INFO(this->get_logger(), "Posicao (%d, %d) esta livre (baixo)", x, y + 1);
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Grid atualizado na posicao (%d, %d): %c", x, y, grid_[y][x]);
        print_grid();  
    }

    void plan_exploration_path()
    {
        if (state_ != RobotState::EXPLORING) {
            RCLCPP_WARN(this->get_logger(), "Tentativa de planejar caminho de exploracao enquanto nao esta no estado EXPLORING.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planejando caminho de exploracao...");

        std::queue<std::pair<Position, std::vector<std::string>>> bfs_queue;
        std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));

        bfs_queue.push({robot_pos_, {}});
        visited[robot_pos_.y][robot_pos_.x] = true;

        bool found = false;
        Position target_pos_found = { -1, -1 };

        while (!bfs_queue.empty() && !found) {
            auto [current_pos, moves] = bfs_queue.front();
            bfs_queue.pop();

            RCLCPP_DEBUG(this->get_logger(), "Checking cell (%d, %d): %c", current_pos.x, current_pos.y, grid_[current_pos.y][current_pos.x]);

            if (grid_[current_pos.y][current_pos.x] == 'U') {
                planned_moves_ = std::queue<std::string>(std::deque<std::string>(moves.begin(), moves.end()));
                RCLCPP_INFO(this->get_logger(), "Caminho para explorar (%d, %d) encontrado com %zu movimentacoes", current_pos.x, current_pos.y, moves.size());
                found = true;
                target_pos_found = current_pos;
                break;
            }

            for (const auto &[dx, dy, direction] : get_neighbors()) {
                Position next_pos = {current_pos.x + dx, current_pos.y + dy};
                if (is_in_bounds(next_pos) && !visited[next_pos.y][next_pos.x] && grid_[next_pos.y][next_pos.x] != 'B') {
                    visited[next_pos.y][next_pos.x] = true;
                    auto new_moves = moves;
                    new_moves.push_back(direction);
                    bfs_queue.push({next_pos, new_moves});
                }
            }
        }

        if (!planned_moves_.empty()) {
            std::string next_direction = planned_moves_.front();
            planned_moves_.pop();
            RCLCPP_INFO(this->get_logger(), "Proximo movimento de exploracao: %s", next_direction.c_str());
            sleep(1);
            send_move_request(next_direction);
        } else if (found) {
            RCLCPP_INFO(this->get_logger(), "Robo já está na posição desconhecida (%d, %d). Marcando como livre.", target_pos_found.x, target_pos_found.y);
            grid_[target_pos_found.y][target_pos_found.x] = 'F';
            plan_exploration_path();
        } else {
            RCLCPP_WARN(this->get_logger(), "Nenhum caminho de exploracao encontrado.");
            if (is_exploration_complete()) {
                RCLCPP_INFO(this->get_logger(), "Exploracao completa. Planejando caminho para o target.");
                state_ = RobotState::MOVING_TO_TARGET;
                RCLCPP_INFO(this->get_logger(), "Estado atualizado para MOVING_TO_TARGET.");
                plan_path_to_target();
            } else {
                RCLCPP_WARN(this->get_logger(), "Exploracao incompleta, mas nenhum caminho encontrado.");
                rclcpp::shutdown();
            }
        }
    }

    void plan_path_to_target()
    {
        if (replan_attempts_ >= max_replan_attempts_) {
            RCLCPP_ERROR(this->get_logger(), "Numero maximo de tentativas de replanejamento atingido. Encerrando nodo.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planejando caminho para o target...");
        plan_path();
        replan_attempts_++;
    }

    void plan_path()
    {
        RCLCPP_INFO(this->get_logger(), "Planejando caminho de (%d, %d) para (%d, %d)", robot_pos_.x, robot_pos_.y, target_pos_.x, target_pos_.y);

        if (target_pos_.x == -1) {
            RCLCPP_WARN(this->get_logger(), "Posicao do target nao localizada.");
            return;
        }

        // Implementacao do BFS para encontrar caminho ao target
        std::queue<std::pair<Position, std::vector<std::string>>> bfs_queue;
        std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));

        bfs_queue.push({robot_pos_, {}});
        visited[robot_pos_.y][robot_pos_.x] = true;

        bool path_found = false;

        while (!bfs_queue.empty() && !path_found) {
            auto [current_pos, moves] = bfs_queue.front();
            bfs_queue.pop();

            if (current_pos == target_pos_) {
                planned_moves_ = std::queue<std::string>(std::deque<std::string>(moves.begin(), moves.end()));
                RCLCPP_INFO(this->get_logger(), "Caminho para o target encontrado com %zu movimentacoes", moves.size());
                path_found = true;
                break;
            }

            for (const auto &[dx, dy, direction] : get_neighbors()) {
                Position neighbor = {current_pos.x + dx, current_pos.y + dy};

                if (is_in_bounds(neighbor) && !visited[neighbor.y][neighbor.x]) {
                    char cell = grid_[neighbor.y][neighbor.x];
                    RCLCPP_DEBUG(this->get_logger(), "Verificando celula (%d, %d): %c", neighbor.x, neighbor.y, cell);

                    if (cell != 'B') {  // Pode mover em todos locais menos cedulas com B
                        visited[neighbor.y][neighbor.x] = true;
                        auto new_moves = moves;
                        new_moves.push_back(direction);
                        bfs_queue.push({neighbor, new_moves});
                    }
                }
            }
        }

        if (path_found) {
            execute_planned_moves();
        } else {
            RCLCPP_WARN(this->get_logger(), "Nenhum caminho para o target localizado.");
            rclcpp::shutdown();
        }
    }

    void execute_planned_moves()
    {
        if (!planned_moves_.empty()) {
            std::string next_direction = planned_moves_.front();
            planned_moves_.pop();
            RCLCPP_INFO(this->get_logger(), "Proximo movimento para o target: %s", next_direction.c_str());
            send_move_request(next_direction);
        } else {
            RCLCPP_INFO(this->get_logger(), "Target alcancado!");
            print_grid();
            rclcpp::shutdown();
        }
    }

    bool is_exploration_complete()
    {
        // Faz BFS para ver se tem algum U ainda
        std::queue<Position> q;
        std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));

        q.push(robot_pos_);
        visited[robot_pos_.y][robot_pos_.x] = true;

        while (!q.empty()) {
            Position current = q.front(); q.pop();

            if (grid_[current.y][current.x] == 'U') {
                return false; // Tem U ainda (que pode ser "chegado")
            }

            for (const auto &[dx, dy, direction] : get_neighbors()) {
                Position neighbor = {current.x + dx, current.y + dy};
                if (is_in_bounds(neighbor) && !visited[neighbor.y][neighbor.x] && grid_[neighbor.y][neighbor.x] != 'B') {
                    q.push(neighbor);
                    visited[neighbor.y][neighbor.x] = true;
                }
            }
        }

        // Sem U's existentes
        return true;
    }

    std::vector<std::tuple<int, int, std::string>> get_neighbors()
    {
        return {
            {1, 0, "right"},
            {0, 1, "down"},
            {-1, 0, "left"},
            {0, -1, "up"}
        };
    }

    Position get_neighbor_position(int x, int y, const std::string &direction)
    {
        if (direction == "left") {
            return {x - 1, y};
        } else if (direction == "right") {
            return {x + 1, y};
        } else if (direction == "up") {
            return {x, y - 1};
        } else if (direction == "down") {
            return {x, y + 1};
        } else {
            // Se a direcao nao existir retorna msm coisa
            return {x, y};
        }
    }

    bool is_in_bounds(const Position &pos)
    {
        return pos.x >= 0 && pos.x < 20 && pos.y >= 0 && pos.y < 20;
    }

    void print_grid()
    {
        // Printa a grid (array 2d)
        std::cout << "Grid state:" << std::endl;
        for (int y = 0; y < 20; ++y) {
            for (int x = 0; x < 20; ++x) {
                std::cout << grid_[y][x] << " ";
            }
            std::cout << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveServiceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();  
    return 0;
}
