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
    // POrra

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

class MoveServiceClient : public rclcpp::Node
{
public:
    MoveServiceClient() : Node("move_service_client")
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        grid_ = std::vector<std::vector<char>>(20, std::vector<char>(20, 'U')); 
        robot_pos_ = {3, 3};  // Arruma depdendno de onde o boneco comeca
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

    void send_initial_move()
    {
        std::vector<std::string> possible_directions = {"right", "down", "left", "up"};
        for (const auto &direction : possible_directions) {
            Position neighbor = get_neighbor_position(robot_pos_.x, robot_pos_.y, direction);
            if (is_in_bounds(neighbor) && grid_[neighbor.y][neighbor.x] != 'B') {
                RCLCPP_INFO(this->get_logger(), "Mandando movimentacao na direcao: %s", direction.c_str());
                send_move_request(direction);
                return;
            }
        }
        RCLCPP_WARN(this->get_logger(), "Sem movimentacao inicial encontrada!");
        rclcpp::shutdown();
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

            if (robot_pos_ == target_pos_) {
                RCLCPP_INFO(this->get_logger(), "Target alcancado!");
                print_grid();
                rclcpp::shutdown();
                return;
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
        }

        if (target_pos_.x != -1) {
            plan_path();
        }

        if (!planned_moves_.empty()) {
            std::string next_direction = planned_moves_.front();
            planned_moves_.pop();
            RCLCPP_INFO(this->get_logger(), "Proximo movimento planejado: %s", next_direction.c_str());
            sleep(1);
            send_move_request(next_direction);
        } else {
            RCLCPP_WARN(this->get_logger(), "Sem caminho, faz a lagosta!");
            print_grid();
            rclcpp::shutdown();
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
        // Opcional
        print_grid();  
    }

    void plan_path()
    {
        RCLCPP_INFO(this->get_logger(), "Planejando caminho de (%d, %d) para (%d, %d)", robot_pos_.x, robot_pos_.y, target_pos_.x, target_pos_.y);

        if (target_pos_.x == -1) {
            RCLCPP_WARN(this->get_logger(), "Posicao do target nao localizada, faz a lagosta.");
            return;
        }

        // Implementacao do BFS
        std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));
        std::queue<std::pair<Position, std::vector<std::string>>> queue;

        queue.push({robot_pos_, {}});
        visited[robot_pos_.y][robot_pos_.x] = true;

        while (!queue.empty()) {
            auto [current_pos, moves] = queue.front();
            queue.pop();

            if (current_pos == target_pos_) {
                planned_moves_ = std::queue<std::string>(std::deque<std::string>(moves.begin(), moves.end()));
                RCLCPP_INFO(this->get_logger(), "Caminho encontrado com %zu movimentacoes", moves.size());
                return;
            }

            for (const auto &[dx, dy, direction] : get_neighbors()) {
                Position neighbor = {current_pos.x + dx, current_pos.y + dy};

                if (is_in_bounds(neighbor) && !visited[neighbor.y][neighbor.x]) {
                    char cell = grid_[neighbor.y][neighbor.x];

                    if (cell != 'B') {  // Pode mover em todos locais menos cedulas com B
                        visited[neighbor.y][neighbor.x] = true;
                        auto new_moves = moves;
                        new_moves.push_back(direction);
                        queue.push({neighbor, new_moves});
                    }
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "Nenhum caminho localizado (reinicie/mude o mapa).");
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
        // For para printar a grid (array 2D)
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
