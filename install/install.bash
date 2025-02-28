#!/bin/bash
# Interactive menu script with colors and external script execution

# ANSI color codes
NC="\e[0m"       # No Color
RED="\e[31m"
GREEN="\e[32m"
YELLOW="\e[33m"
BLUE="\e[34m"
CYAN="\e[36m"
BOLD="\e[1m"

# Function to display the menu
show_menu() {
    echo -e "${BOLD}${CYAN}-----------------------------${NC}"
    echo -e "${BOLD}${GREEN}        MAIN MENU            ${NC}"
    echo -e "${BOLD}${CYAN}-----------------------------${NC}"
    echo -e "${YELLOW}1.${NC} Install ROS2"
    echo -e "${YELLOW}2.${NC} Install Docker"
    echo -e "${YELLOW}3.${NC} Update marrtinorobot_ws"
    echo -e "${YELLOW}4.${NC} Update System Packages"
    echo -e "${YELLOW}5.${NC} Run External Script"
    echo -e "${RED}6.${NC} Exit"
    echo -e "${BOLD}${CYAN}-----------------------------${NC}"
}

# Confirmation function
confirm_action() {
    read -p "Are you sure you want to proceed? [y/n]: " confirm
    case "$confirm" in
        y|Y) return 0 ;;  # Confirmed
        n|N) echo -e "${RED}Action canceled.${NC}"; return 1 ;;  # Canceled
        *) echo -e "${RED}Invalid input. Action canceled.${NC}"; return 1 ;;
    esac
}

# Functions for each option
Sub_1() {
    confirm_action || return
    echo -e "${GREEN}Installing ROS2...${NC}"
    # Add ROS2 installation commands here
    sleep 2
    echo -e "${GREEN}ROS2 installation completed!${NC}"
}

Sub_2() {
    confirm_action || return
    echo -e "${GREEN}Installing Docker...${NC}"
    # Add Docker installation commands here
    sleep 2
    echo -e "${GREEN}Docker installation completed!${NC}"
}

Sub_3() {
    confirm_action || return
    echo -e "${GREEN}Updating marrtinorobot_ws...${NC}"
    # Add workspace update commands here
    sleep 2
    echo -e "${GREEN}marrtinorobot_ws updated!${NC}"
}

Sub_4() {
    confirm_action || return
    echo -e "${GREEN}Updating system packages...${NC}"
    sudo apt update && sudo apt upgrade -y
    echo -e "${GREEN}System packages updated successfully!${NC}"
}

Sub_5() {
    read -p "Enter the path to the external script: " script_path
    if [[ -x "$script_path" ]]; then
        echo -e "${BLUE}Executing external script: ${script_path}${NC}"
        bash "$script_path"
        echo -e "${GREEN}External script execution completed. Returning to menu...${NC}"
    else
        echo -e "${RED}Error: Script not found or not executable. Please check the path.${NC}"
    fi
}

# Main menu loop
while true; do
    show_menu
    read -p "Choose an option [1-6]: " choice

    case $choice in
        1) Sub_1 ;;
        2) Sub_2 ;;
        3) Sub_3 ;;
        4) Sub_4 ;;
        5) Sub_5 ;;
        6) echo -e "${RED}Exiting the program...${NC}"; exit 0 ;;
        *) echo -e "${RED}Invalid option. Please try again.${NC}" ;;
    esac

    echo -e "${CYAN}Press ENTER to continue...${NC}"
    read   # Pause before showing the menu again
done
