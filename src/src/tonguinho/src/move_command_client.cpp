#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <vector>
#include <queue>
#include <string>
#include <array>
#include <iostream>

using namespace std::chrono_literals;

class MoveServiceClient : public rclcpp::Node
{
public:
    MoveServiceClient() : Node("move_service_client")
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        grid_ = std::vector<std::vector<char>>(20, std::vector<char>(20, 'U'));  // 'U' for unknown cells
        robot_pos_ = {3, 3};  // Set the starting position to (3, 3)
        target_pos_ = {-1, -1};  // Target position (unknown until first response)
        last_direction_ = "";

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service '/move_command'...");
        }

        // Initialize the grid at the robot's starting position
        grid_[robot_pos_[0]][robot_pos_[1]] = 'R';

        // Send an initial move to obtain the target position
        send_initial_move();
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    std::vector<std::vector<char>> grid_;
    std::array<int, 2> robot_pos_;
    std::array<int, 2> target_pos_;
    std::queue<std::string> planned_moves_;
    std::string last_direction_;

    void send_initial_move()
    {
        // Try moving in a valid direction to get the target position
        std::vector<std::string> possible_directions = {"right", "down", "left", "up"};
        for (const auto &direction : possible_directions) {
            std::array<int, 2> neighbor = get_neighbor_position(robot_pos_[0], robot_pos_[1], direction);
            if (is_in_bounds(neighbor)) {
                send_move_request(direction);
                return;
            }
        }
        RCLCPP_WARN(this->get_logger(), "No valid initial move found!");
        rclcpp::shutdown();
    }

    void send_move_request(const std::string &direction)
    {
        last_direction_ = direction;  // Store the last attempted direction

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future_result = client_->async_send_request(request,
            std::bind(&MoveServiceClient::process_response, this, std::placeholders::_1));
    }

    void process_response(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture response_future)
    {
        auto response = response_future.get();

        if (response->success) {
            // Mark previous position as free
            grid_[robot_pos_[0]][robot_pos_[1]] = 'F';

            robot_pos_ = {response->robot_pos[0], response->robot_pos[1]};
            grid_[robot_pos_[0]][robot_pos_[1]] = 'R';

            // Set target position if unknown
            if (target_pos_[0] == -1) {
                target_pos_ = {response->target_pos[0], response->target_pos[1]};
                grid_[target_pos_[0]][target_pos_[1]] = 'T';
            }

            update_grid(response);

            if (robot_pos_ == target_pos_) {
                RCLCPP_INFO(this->get_logger(), "Target reached!");
                print_grid();
                rclcpp::shutdown();
                return;
            }

        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to move.");

            // Update the grid to mark the attempted cell as blocked
            int x = robot_pos_[0];
            int y = robot_pos_[1];

            std::array<int, 2> blocked_pos = get_neighbor_position(x, y, last_direction_);
            if (is_in_bounds(blocked_pos)) {
                grid_[blocked_pos[0]][blocked_pos[1]] = 'B';
            }
        }

        // Plan path if queue is empty and target position is known
        if (planned_moves_.empty() && target_pos_[0] != -1) {
            plan_path();
        }

        // Execute next planned move
        if (!planned_moves_.empty()) {
            std::string next_direction = planned_moves_.front();
            planned_moves_.pop();
            send_move_request(next_direction);
        } else if (target_pos_[0] != -1) {
            RCLCPP_WARN(this->get_logger(), "No available path!");
            print_grid();
            rclcpp::shutdown();
        } else {
            // Wait for target position to be known
            send_initial_move();  // Try another move to get the target position
        }
    }

    void update_grid(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response> &response)
    {
        int x = robot_pos_[0];
        int y = robot_pos_[1];

        // Left
        if (is_in_bounds({x, y - 1})) {
            if (response->left == "b") {
                grid_[x][y - 1] = 'B';
            } else if (grid_[x][y - 1] == 'U') {
                grid_[x][y - 1] = 'F';
            }
        }

        // Right
        if (is_in_bounds({x, y + 1})) {
            if (response->right == "b") {
                grid_[x][y + 1] = 'B';
            } else if (grid_[x][y + 1] == 'U') {
                grid_[x][y + 1] = 'F';
            }
        }

        // Up
        if (is_in_bounds({x - 1, y})) {
            if (response->up == "b") {
                grid_[x - 1][y] = 'B';
            } else if (grid_[x - 1][y] == 'U') {
                grid_[x - 1][y] = 'F';
            }
        }

        // Down
        if (is_in_bounds({x + 1, y})) {
            if (response->down == "b") {
                grid_[x + 1][y] = 'B';
            } else if (grid_[x + 1][y] == 'U') {
                grid_[x + 1][y] = 'F';
            }
        }
    }

    void plan_path()
    {
        // Ensure target position is known
        if (target_pos_[0] == -1) {
            RCLCPP_WARN(this->get_logger(), "Target position unknown, cannot plan path.");
            return;
        }

        // Implement BFS with exploration of unknown cells
        std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));
        std::queue<std::pair<std::array<int, 2>, std::queue<std::string>>> queue;

        queue.push({robot_pos_, {}});
        visited[robot_pos_[0]][robot_pos_[1]] = true;

        while (!queue.empty()) {
            auto [current_pos, moves] = queue.front();
            queue.pop();

            if (current_pos == target_pos_) {
                planned_moves_ = moves;
                return;
            }

            for (const auto &[dx, dy, direction] : get_neighbors()) {
                std::array<int, 2> neighbor = {current_pos[0] + dx, current_pos[1] + dy};

                if (is_in_bounds(neighbor) && !visited[neighbor[0]][neighbor[1]]) {
                    char cell = grid_[neighbor[0]][neighbor[1]];

                    if (cell != 'B') {  // We can move into 'F', 'U', 'T', 'R'
                        visited[neighbor[0]][neighbor[1]] = true;
                        auto new_moves = moves;
                        new_moves.push(direction);
                        queue.push({neighbor, new_moves});
                    }
                }
            }
        }

        // If no path found
        RCLCPP_WARN(this->get_logger(), "No path to target found during planning.");
    }

    std::vector<std::tuple<int, int, std::string>> get_neighbors()
    {
        return {
            {0, 1, "right"},
            {1, 0, "down"},
            {0, -1, "left"},
            {-1, 0, "up"}
        };
    }

    std::array<int, 2> get_neighbor_position(int x, int y, const std::string &direction)
    {
        if (direction == "left") {
            return {x, y - 1};
        } else if (direction == "right") {
            return {x, y + 1};
        } else if (direction == "up") {
            return {x - 1, y};
        } else if (direction == "down") {
            return {x + 1, y};
        } else {
            // Invalid direction
            return {x, y};
        }
    }

    bool is_in_bounds(const std::array<int, 2> &pos)
    {
        return pos[0] >= 0 && pos[0] < 20 && pos[1] >= 0 && pos[1] < 20;
    }

    void print_grid()
    {
        for (const auto &row : grid_) {
            for (const auto &cell : row) {
                std::cout << cell << " ";
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
    return 0;
}
