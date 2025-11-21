class Floor:   
    def __init__(self, floor_number, total_floors):
        self.floor_number = floor_number
        self.up_button_pressed = False
        self.down_button_pressed = False

        # Button availability based on position in building
        if floor_number == 0:  # Ground floor
            self.has_up_button = True
            self.has_down_button = False
        elif floor_number == total_floors - 1:  # Top floor
            self.has_up_button = False
            self.has_down_button = True
        else:  # Middle floors
            self.has_up_button = True
            self.has_down_button = True

    def press_up(self):
        """Someone presses the UP button on this floor"""
        if not self.has_up_button:
            return False
        self.up_button_pressed = True
        return True

    def press_down(self):
        """Someone presses the DOWN button on this floor"""
        if not self.has_down_button:
            return False
        self.down_button_pressed = True
        return True

    def clear_request(self, direction):
        """Clear the button after elevator arrives"""
        if direction == "UP":
            self.up_button_pressed = False
        elif direction == "DOWN":
            self.down_button_pressed = False

    def __str__(self):
        up = "ON" if self.up_button_pressed else "OFF"
        down = "ON" if self.down_button_pressed else "OFF"
        return f"Floor {self.floor_number}: UP={up}, DOWN={down}"   # ✅ FIXED: Changed 'off' to 'down'



#Models a person. Tracks their journey (waiting → in elevator → done).
class Passenger:
    """
    Simple passenger model:
    - WAITING on a floor
    - IN_ELEVATOR (assigned to an elevator)
    - DONE (reached destination)
    """

    def __init__(self, passenger_id, start_floor, destination_floor):
        self.id = passenger_id
        self.start_floor = start_floor
        self.destination_floor = destination_floor
        self.state = "WAITING"      # WAITING, IN_ELEVATOR, DONE
        self.elevator_id = None     # which elevator, if any

    def direction(self):
        if self.destination_floor > self.start_floor:
            return "UP"
        elif self.destination_floor < self.start_floor:
            return "DOWN"
        return "NONE"  # same floor (degenerate case)

    def __str__(self):
        return (f"Passenger {self.id}: {self.start_floor} -> {self.destination_floor} "
                f"({self.state}, elevator={self.elevator_id})")


class Elevator:
    def __init__(self, elevator_id, total_floors):
        self.elevator_id = elevator_id
        self.current_floor = 0  # Start at ground floor
        self.direction = "IDLE"  # "IDLE", "MOVING_UP", "MOVING_DOWN"
        self.destination_queue = []  # List of floors to visit
        self.internal_buttons = [False] * total_floors  # Button states inside elevator
        self.door_open = False  # Simple door state
        self.just_stopped = False  # True if stopped at a floor this tick

    def open_door(self):
        self.door_open = True

    def close_door(self):
        self.door_open = False

    def add_destination(self, floor_num):
        """Add a floor to the elevator's destination list"""
        if floor_num == self.current_floor:
            return f"Elevator {self.elevator_id}: Already at floor {floor_num}"
            
        if floor_num not in self.destination_queue:
            self.destination_queue.append(floor_num)
            # Mark internal button
            if 0 <= floor_num < len(self.internal_buttons):
                self.internal_buttons[floor_num] = True

            # SMART SORTING: Consider current position and direction
            if len(self.destination_queue) > 1:
                if self.direction in ("MOVING_UP", "IDLE"):
                    # Sort destinations that are above current floor in ascending order
                    above_current = [f for f in self.destination_queue if f > self.current_floor]
                    below_current = [f for f in self.destination_queue if f < self.current_floor]
                    above_current.sort()
                    below_current.sort(reverse=True)  # Handle below floors in descending order
                    self.destination_queue = above_current + below_current
                    
                elif self.direction == "MOVING_DOWN":
                    # Sort destinations that are below current floor in descending order  
                    above_current = [f for f in self.destination_queue if f > self.current_floor]
                    below_current = [f for f in self.destination_queue if f < self.current_floor]
                    below_current.sort(reverse=True)
                    above_current.sort()  # Handle above floors in ascending order
                    self.destination_queue = below_current + above_current

            return f"Elevator {self.elevator_id}: Added floor {floor_num} to destination queue"
        return f"Elevator {self.elevator_id}: Floor {floor_num} already in destination queue"

    def move(self):
        """
        Move the elevator one step based on its current direction and destinations.
        One tick of the simulation for this elevator.
        """
        self.just_stopped = False  # reset flag at each tick

        if not self.destination_queue:
            self.direction = "IDLE"
            self.close_door()
            return None

        # Next destination to aim for
        next_destination = self.destination_queue[0]

        # If already at destination: open/close doors, clear internal button & queue
        if self.current_floor == next_destination:
            self.open_door()
            self.just_stopped = True
            # Clear internal button
            if 0 <= self.current_floor < len(self.internal_buttons):
                self.internal_buttons[self.current_floor] = False
            # Remove this destination
            self.destination_queue.pop(0)
            result = (
                f"Elevator {self.elevator_id}: Stopped at floor {self.current_floor} "
                f"(Doors opened)"
            )
            # After servicing, update direction for next destination
            self.update_direction()
            # Close the door at the end of the tick for simplicity
            self.close_door()
            return result

        # Not at destination yet: move one floor toward it
        self.close_door()

        if next_destination > self.current_floor:
            self.direction = "MOVING_UP"
            self.current_floor += 1
            return f"Elevator {self.elevator_id}: Moving UP to floor {self.current_floor}"
        elif next_destination < self.current_floor:
            self.direction = "MOVING_DOWN"
            self.current_floor -= 1
            return f"Elevator {self.elevator_id}: Moving DOWN to floor {self.current_floor}"

        return None

    def update_direction(self):
        """Update the elevator's direction based on its destination queue"""
        if not self.destination_queue:
            self.direction = "IDLE"
        else:
            # Look at ALL destinations to make better direction decisions
            has_above = any(f > self.current_floor for f in self.destination_queue)
            has_below = any(f < self.current_floor for f in self.destination_queue)
            
            if self.direction == "MOVING_UP" and has_above:
                self.direction = "MOVING_UP"
            elif self.direction == "MOVING_DOWN" and has_below:
                self.direction = "MOVING_DOWN"  
            elif has_above:
                self.direction = "MOVING_UP"
            elif has_below:
                self.direction = "MOVING_DOWN"
            else:
                self.direction = "IDLE"

    def __str__(self):
        return (
            f"Elevator {self.elevator_id}: Floor {self.current_floor} "
            f"({self.direction}), Destinations={self.destination_queue}"
        )


class Building:
    def __init__(self, num_floors, num_elevators):
        self.num_floors = num_floors
        self.num_elevators = num_elevators

        # Create all the floors
        self.floors = [
            Floor(floor_num, num_floors) for floor_num in range(num_floors)
        ]

        # Create all the elevators
        self.elevators = [
            Elevator(elevator_id, num_floors) for elevator_id in range(num_elevators)
        ]

    def get_floor(self, floor_number):
        """Get a specific floor object"""
        if 0 <= floor_number < self.num_floors:
            return self.floors[floor_number]
        return None

    def get_elevator(self, elevator_id):
        """Get a specific elevator object"""
        if 0 <= elevator_id < self.num_elevators:
            return self.elevators[elevator_id]
        return None


class ElevatorController:
    def __init__(self, building):
        self.building = building
        # List of (floor_num, direction, request_id)
        self.pending_requests = []
        self.request_counter = 0

        # Passenger-related
        self.passenger_counter = 0
        self.waiting_passengers = {f: [] for f in range(building.num_floors)}
        self.onboard_passengers = {
            e.elevator_id: [] for e in building.elevators
        }
        self.completed_passengers = []

    # ---- Requests from floors ----
    def request_elevator(self, floor_num, direction):
        """Someone on a floor presses UP or DOWN button"""
        floor = self.building.get_floor(floor_num)
        if not floor:
            return f"CONTROLLER: Floor {floor_num} does not exist"

        # Respect which buttons this floor has
        if direction == "UP":
            ok = floor.press_up()
        else:
            ok = floor.press_down()

        if not ok:
            return (
                f"CONTROLLER: Floor {floor_num} has no {direction} button "
                f"(request ignored)"
            )

        self.request_counter += 1
        request_id = self.request_counter
        request = (floor_num, direction, request_id)

        if request not in self.pending_requests:
            self.pending_requests.append(request)

        return (
            f"CONTROLLER: Floor {floor_num} requests elevator going {direction} "
            f"(Request #{request_id})"
        )

    # ---- Requests from inside elevators ----
    def add_elevator_destination(self, elevator_id, floor_num):
        """Someone inside an elevator presses a floor button"""
        elevator = self.building.get_elevator(elevator_id)
        if elevator:
            result = elevator.add_destination(floor_num)
            return f"CONTROLLER: {result}"
        return f"CONTROLLER: Elevator {elevator_id} does not exist"

    # ---- Passengers ----
    def create_passenger(self, start_floor, destination_floor):
        """Create a passenger and make them call an elevator."""
        if not (0 <= start_floor < self.building.num_floors):
            return "PASSENGER: Invalid start floor"
        if not (0 <= destination_floor < self.building.num_floors):
            return "PASSENGER: Invalid destination floor"
        if start_floor == destination_floor:
            return "PASSENGER: Start and destination are same"

        self.passenger_counter += 1
        p = Passenger(self.passenger_counter, start_floor, destination_floor)
        self.waiting_passengers[start_floor].append(p)

        # Passenger automatically presses the correct button
        direction = p.direction()
        request_msg = self.request_elevator(start_floor, direction)

        return (
            f"PASSENGER: Created {p} and requested elevator ({direction})\n"
            f"{request_msg}"
        )

    def handle_passengers_at_stop(self, elevator, log_list):
        """
        When an elevator stops at a floor:
        - drop off passengers whose destination is this floor
        - pick up waiting passengers from this floor
        """
        floor_num = elevator.current_floor

        # 1) Drop off passengers who reached their destination
        onboard = self.onboard_passengers[elevator.elevator_id]
        remaining_onboard = []
        for p in onboard:
            if p.destination_floor == floor_num:
                p.state = "DONE"
                p.elevator_id = None
                self.completed_passengers.append(p)
                log_list.append(
                    f"PASSENGER: Passenger {p.id} exited Elevator {elevator.elevator_id} at floor {floor_num}"
                )
            else:
                remaining_onboard.append(p)
        self.onboard_passengers[elevator.elevator_id] = remaining_onboard

        # 2) Pick up waiting passengers from this floor
        waiting_here = self.waiting_passengers[floor_num]
        remaining_waiting = []

        for p in waiting_here:
            # Simple rule: board all waiting passengers at this floor
            p.state = "IN_ELEVATOR"
            p.elevator_id = elevator.elevator_id
            self.onboard_passengers[elevator.elevator_id].append(p)
            log_list.append(
                f"PASSENGER: Passenger {p.id} boarded Elevator {elevator.elevator_id} at floor {floor_num}, "
                f"destination {p.destination_floor}"
            )
            # ensure their destination is in the elevator queue
            elevator.add_destination(p.destination_floor)  # ✅ FIXED: Changed 'destination_force_floor' to 'destination_floor'

        # All waiting passengers boarded (for this simple model)
        self.waiting_passengers[floor_num] = remaining_waiting

    # ---- One simulation tick ----
    def step(self):
        """
        Run one tick of the simulation:
        - Dispatch pending requests (brain)
        - Move all elevators one floor
        - Handle passengers boarding/exiting when elevators stop
        """
        results = []

        # First, assign elevators to floor requests
        dispatch_results = self.dispatch()
        if dispatch_results:
            results.extend(dispatch_results)

        # Then move all elevators one floor
        for elevator in self.building.elevators:
            move_result = elevator.move()
            if move_result:
                results.append(move_result)

            # If elevator stopped this tick, handle passengers
            if elevator.just_stopped:
                self.handle_passengers_at_stop(elevator, results)

        return results

    def dispatch(self):
        """THE BRAIN: Decide which elevator should handle which request (oldest first)"""
        if not self.pending_requests:
            return []

        results = []

        # Oldest requests first (by request_id)
        self.pending_requests.sort(key=lambda x: x[2])
        requests_to_process = list(self.pending_requests)

        for request in requests_to_process:
            floor_num, direction, request_id = request

            if request not in self.pending_requests:
                continue

            best_elevator = self.find_best_elevator(floor_num, direction)

            if best_elevator:
                # Skip if elevator is already at that floor and idle
                if (best_elevator.current_floor == floor_num and 
                    best_elevator.direction == "IDLE"):
                    # Clear the request without moving elevator
                    floor = self.building.get_floor(floor_num)
                    if floor:
                        floor.clear_request(direction)
                    self.pending_requests.remove(request)
                    results.append(
                        f"DISPATCH: Elevator {best_elevator.elevator_id} already at floor {floor_num} - request cleared"
                    )
                    continue
                    
                results.append(
                    f"DISPATCH: Sending Elevator {best_elevator.elevator_id} to "
                    f"floor {floor_num} for {direction} request (Request #{request_id})"
                )
                best_elevator.add_destination(floor_num)

                # Clear the floor request
                floor = self.building.get_floor(floor_num)
                if floor:
                    floor.clear_request(direction)

                # Remove this request from pending
                self.pending_requests.remove(request)

        return results

    def find_best_elevator(self, floor_num, direction):
        """Choose the best elevator (simple scoring)"""
        best_elevator = None
        best_score = float("inf")

        for elevator in self.building.elevators:
            score = self.calculate_elevator_score(elevator, floor_num, direction)
            if score < best_score:
                best_score = score
                best_elevator = elevator

        return best_elevator

    def calculate_elevator_score(self, elevator, floor_num, direction):
        """Lower score = better"""
        distance = abs(elevator.current_floor - floor_num)
        score = distance

        # STRONG penalty if moving in wrong direction
        if elevator.direction == "MOVING_UP" and direction == "DOWN":
            score += 100  # Increased penalty
        elif elevator.direction == "MOVING_DOWN" and direction == "UP":
            score += 100  # Increased penalty

        # Penalty for busy elevators
        score += len(elevator.destination_queue) * 5

        # Bonus for idle elevators
        if elevator.direction == "IDLE":
            score -= 20  # Increased bonus

        # STRONG BONUS for same direction AND on the way
        if (elevator.direction == "MOVING_UP" and direction == "UP"
            and elevator.current_floor < floor_num):
            score -= 15  # Strong bonus
        elif (elevator.direction == "MOVING_DOWN" and direction == "DOWN" 
              and elevator.current_floor > floor_num):
            score -= 15  # Strong bonus

        # Perfect match: already at floor and idle
        if elevator.current_floor == floor_num and elevator.direction == "IDLE":
            score = -100  # Strong preference

        return score

    def get_status(self):
        """Get the current state of all elevators"""
        return [
            f"[Elevator {e.elevator_id}: Floor {e.current_floor} "
            f"({e.direction}) -> Destinations: {e.destination_queue}]"
            for e in self.building.elevators
        ]

    def get_pending_requests(self):
        """Get list of pending floor requests"""
        return [
            f"Floor {f} {d} (Request #{rid})"
            for (f, d, rid) in self.pending_requests
        ]

    def get_passenger_status(self):
        """Return a simple summary of passengers."""
        lines = []

        for floor, plist in self.waiting_passengers.items():
            for p in plist:
                lines.append(f"[WAITING] {p}")
        for eid, plist in self.onboard_passengers.items():
            for p in plist:
                lines.append(f"[IN_ELEVATOR] {p}")
        for p in self.completed_passengers:
            lines.append(f"[DONE] {p}")

        if not lines:
            lines.append("No passengers in system.")
        return lines


# ---------- CLI helpers ----------

def get_positive_int(prompt):
    """Ask for a positive integer (>= 1), no defaults."""
    while True:
        value = input(prompt).strip()
        try:
            num = int(value)
            if num < 1:
                print("Please enter a number greater than 0.")
                continue
            return num
        except ValueError:
            print("Please enter a valid integer number.")


# ---------- Main CLI ----------

def main():
    """Main CLI interface for the elevator system"""
    print("=" * 60)
    print("ELEVATOR CONTROL SYSTEM - INTERACTIVE MODE")
    print("=" * 60)

    # Get building configuration from user (no defaults)
    num_floors = get_positive_int("Enter number of floors: ")
    num_elevators = get_positive_int("Enter number of elevators: ")

    building = Building(num_floors, num_elevators)
    controller = ElevatorController(building)

    print(
        f"\nBuilding created with {num_floors} floors and "
        f"{num_elevators} elevators"
    )
    print("\nAvailable commands:")
    print("  call <floor> <up/down>    - Call elevator from floor (e.g., 'call 3 up')")
    print("  press <elevator> <floor>  - Press button inside elevator (e.g., 'press 0 5')")
    print("  passenger <start> <dest>  - Create a passenger (e.g., 'passenger 0 4')")
    print("  step                      - Advance simulation by one tick")
    print("  auto <steps>              - Run multiple steps automatically")
    print("  status                    - Show current elevator status")
    print("  requests                  - Show pending floor requests")
    print("  pstatus                   - Show passenger status")
    print("  help                      - Show commands again")
    print("  quit                      - Exit")

    step_count = 0

    while True:
        command = input(f"\n[{step_count}] Enter command: ").strip().lower()

        if command in ("quit", "exit"):
            print("Exiting Elevator Control System. Goodbye!")
            break

        elif command == "help":
            print("\nCommands:")
            print("  call <floor> <up/down>")
            print("  press <elevator> <floor>")
            print("  passenger <start> <dest>")
            print("  step")
            print("  auto <steps>")
            print("  status")
            print("  requests")
            print("  pstatus")
            print("  quit")

        elif command == "status":
            print("\nCurrent elevator status:")
            for s in controller.get_status():
                print(" ", s)
            print("\nFloors:")
            for floor in building.floors:
                print(" ", floor)

        elif command == "requests":
            pending = controller.get_pending_requests()
            if pending:
                print("\nPending floor requests:")
                for r in pending:
                    print(" ", r)
            else:
                print("\nNo pending floor requests.")

        elif command == "pstatus":
            print("\nPassenger status:")
            for line in controller.get_passenger_status():
                print(" ", line)

        elif command.startswith("call "):
            parts = command.split()
            if len(parts) == 3:
                try:
                    floor = int(parts[1])
                    direction = parts[2].upper()
                    if direction not in ("UP", "DOWN"):
                        print("Direction must be 'up' or 'down'")
                        continue
                    if not (0 <= floor < num_floors):
                        print(f"Floor must be between 0 and {num_floors - 1}")
                        continue
                    res = controller.request_elevator(floor, direction)
                    print(res)
                except ValueError:
                    print("Invalid floor number.")
            else:
                print("Usage: call <floor> <up/down>")

        elif command.startswith("press "):
            parts = command.split()
            if len(parts) == 3:
                try:
                    elevator_id = int(parts[1])
                    floor = int(parts[2])
                    if not (0 <= elevator_id < num_elevators):
                        print(f"Elevator ID must be between 0 and {num_elevators - 1}")
                        continue
                    if not (0 <= floor < num_floors):
                        print(f"Floor must be between 0 and {num_floors - 1}")
                        continue
                    res = controller.add_elevator_destination(elevator_id, floor)
                    print(res)
                except ValueError:
                    print("Invalid elevator or floor number.")
            else:
                print("Usage: press <elevator> <floor>")

        elif command.startswith("passenger "):
            parts = command.split()
            if len(parts) == 3:
                try:
                    start = int(parts[1])
                    dest = int(parts[2])
                    res = controller.create_passenger(start, dest)
                    print(res)
                except ValueError:
                    print("Invalid floor numbers for passenger.")
            else:
                print("Usage: passenger <start_floor> <destination_floor>")

        elif command == "step":
            step_count += 1
            print(f"\n--- TICK {step_count} ---")
            results = controller.step()
            if results:
                for r in results:
                    print(" ", r)
            else:
                print(" No movement this tick.")

        elif command.startswith("auto "):
            parts = command.split()
            if len(parts) == 2:
                try:
                    steps = int(parts[1])
                    if steps < 1:
                        print("Steps must be at least 1.")
                        continue
                    for _ in range(steps):
                        step_count += 1
                        print(f"\n--- TICK {step_count} ---")
                        results = controller.step()
                        if results:
                            for r in results:
                                print(" ", r)
                        else:
                            print(" No movement this tick.")
                except ValueError:
                    print("Invalid number of steps.")
            else:
                print("Usage: auto <steps>")

        else:
            print("Unknown command. Type 'help' to see available commands.")


if __name__ == "__main__":
    main()