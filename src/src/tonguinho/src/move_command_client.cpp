#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <vector>
#include <queue>
#include <string>
#include <array>
#include <iostream>

using namespace std::chrono_literals;

struct Position {
    int x;  // Column index
    int y;  // Row index

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

        grid_ = std::vector<std::vector<char>>(20, std::vector<char>(20, 'U'));  // 'U' for unknown cells
        robot_pos_ = {3, 3};  // Set the starting position to (3, 3)
        target_pos_ = {-1, -1};  // Target position (unknown until first response)
        last_direction_ = "";

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service '/move_command'...");
        }

        // Initialize the grid at the robot's starting position
        grid_[robot_pos_.y][robot_pos_.x] = 'R';

        // Send an initial move to obtain the target position
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
        // Try moving in a valid direction to get the target position
        std::vector<std::string> possible_directions = {"right", "down", "left", "up"};
        for (const auto &direction : possible_directions) {
            Position neighbor = get_neighbor_position(robot_pos_.x, robot_pos_.y, direction);
            if (is_in_bounds(neighbor) && grid_[neighbor.y][neighbor.x] != 'B') {
                RCLCPP_INFO(this->get_logger(), "Sending initial move in direction: %s", direction.c_str());
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

        RCLCPP_INFO(this->get_logger(), "Attempting to move %s from position (%d, %d)", direction.c_str(), robot_pos_.x, robot_pos_.y);

        auto future_result = client_->async_send_request(request,
            std::bind(&MoveServiceClient::process_response, this, std::placeholders::_1));
    }

    void process_response(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture response_future)
    {
        auto response = response_future.get();

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Move %s successful", last_direction_.c_str());

            // Mark previous position as free
            grid_[robot_pos_.y][robot_pos_.x] = 'F';

            // Update robot's position
            robot_pos_.x = response->robot_pos[1];  // Swap indices if service uses (y, x)
            robot_pos_.y = response->robot_pos[0];
            grid_[robot_pos_.y][robot_pos_.x] = 'R';

            RCLCPP_INFO(this->get_logger(), "Robot moved to position (%d, %d)", robot_pos_.x, robot_pos_.y);

            // Set target position if unknown
            if (target_pos_.x == -1) {
                target_pos_.x = response->target_pos[1];
                target_pos_.y = response->target_pos[0];
                grid_[target_pos_.y][target_pos_.x] = 'T';
                RCLCPP_INFO(this->get_logger(), "Target position identified at (%d, %d)", target_pos_.x, target_pos_.y);
            }

            update_grid(response);

            if (robot_pos_ == target_pos_) {
                RCLCPP_INFO(this->get_logger(), "Target reached!");
                print_grid();
                rclcpp::shutdown();
                return;
            }

        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to move %s from position (%d, %d)", last_direction_.c_str(), robot_pos_.x, robot_pos_.y);

            // Update the grid to mark the attempted cell as blocked
            Position blocked_pos = get_neighbor_position(robot_pos_.x, robot_pos_.y, last_direction_);
            if (is_in_bounds(blocked_pos)) {
                grid_[blocked_pos.y][blocked_pos.x] = 'B';
                RCLCPP_INFO(this->get_logger(), "Marked position (%d, %d) as blocked", blocked_pos.x, blocked_pos.y);
            }

            // Clear planned moves and replan path
            planned_moves_ = std::queue<std::string>();  // Reset the queue
            RCLCPP_INFO(this->get_logger(), "Cleared planned moves due to failed move. Replanning path...");
        }

        // Plan path if target position is known
        if (target_pos_.x != -1) {
            plan_path();
        }

        // Execute next planned move
        if (!planned_moves_.empty()) {
            std::string next_direction = planned_moves_.front();
            planned_moves_.pop();
            RCLCPP_INFO(this->get_logger(), "Next planned move: %s", next_direction.c_str());
            sleep(1);
            send_move_request(next_direction);
        } else {
            RCLCPP_WARN(this->get_logger(), "No available path!");
            print_grid();
            rclcpp::shutdown();
        }
    }

    void update_grid(const std::shared_ptr<cg_interfaces::srv::MoveCmd::Response> &response)
    {
        int x = robot_pos_.x;
        int y = robot_pos_.y;

        RCLCPP_INFO(this->get_logger(), "Updating grid based on sensor data...");

        // Left
        if (is_in_bounds({x - 1, y})) {
            if (response->left == "b") {
                grid_[y][x - 1] = 'B';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is blocked (left)", x - 1, y);
            } else if (grid_[y][x - 1] == 'U') {
                grid_[y][x - 1] = 'F';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is free (left)", x - 1, y);
            }
        }

        // Right
        if (is_in_bounds({x + 1, y})) {
            if (response->right == "b") {
                grid_[y][x + 1] = 'B';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is blocked (right)", x + 1, y);
            } else if (grid_[y][x + 1] == 'U') {
                grid_[y][x + 1] = 'F';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is free (right)", x + 1, y);
            }
        }

        // Up
        if (is_in_bounds({x, y - 1})) {
            if (response->up == "b") {
                grid_[y - 1][x] = 'B';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is blocked (up)", x, y - 1);
            } else if (grid_[y - 1][x] == 'U') {
                grid_[y - 1][x] = 'F';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is free (up)", x, y - 1);
            }
        }

        // Down
        if (is_in_bounds({x, y + 1})) {
            if (response->down == "b") {
                grid_[y + 1][x] = 'B';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is blocked (down)", x, y + 1);
            } else if (grid_[y + 1][x] == 'U') {
                grid_[y + 1][x] = 'F';
                RCLCPP_INFO(this->get_logger(), "Position (%d, %d) is free (down)", x, y + 1);
            }
        }

        print_grid();  // Optionally print the grid after each update
    }

    void plan_path()
    {
        RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)", robot_pos_.x, robot_pos_.y, target_pos_.x, target_pos_.y);

        // Ensure target position is known
        if (target_pos_.x == -1) {
            RCLCPP_WARN(this->get_logger(), "Target position unknown, cannot plan path.");
            return;
        }

        // Implement BFS with exploration of unknown cells
        std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));
        std::queue<std::pair<Position, std::vector<std::string>>> queue;

        queue.push({robot_pos_, {}});
        visited[robot_pos_.y][robot_pos_.x] = true;

        while (!queue.empty()) {
            auto [current_pos, moves] = queue.front();
            queue.pop();

            if (current_pos == target_pos_) {
                // Convert vector of moves to queue
                planned_moves_ = std::queue<std::string>(std::deque<std::string>(moves.begin(), moves.end()));
                RCLCPP_INFO(this->get_logger(), "Path found with %zu moves", moves.size());
                return;
            }

            for (const auto &[dx, dy, direction] : get_neighbors()) {
                Position neighbor = {current_pos.x + dx, current_pos.y + dy};

                if (is_in_bounds(neighbor) && !visited[neighbor.y][neighbor.x]) {
                    char cell = grid_[neighbor.y][neighbor.x];

                    if (cell != 'B') {  // We can move into 'F', 'U', 'T', 'R'
                        visited[neighbor.y][neighbor.x] = true;
                        auto new_moves = moves;
                        new_moves.push_back(direction);
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
            // Invalid direction
            return {x, y};
        }
    }

    bool is_in_bounds(const Position &pos)
    {
        return pos.x >= 0 && pos.x < 20 && pos.y >= 0 && pos.y < 20;
    }

    void print_grid()
    {
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
    rclcpp::shutdown();  // Ensure proper shutdown
    return 0;
}
