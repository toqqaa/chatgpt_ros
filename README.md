# chatgpt_ros

`chatgpt_ros` is a ROS package that integrates OpenAI's ChatGPT API into the Robot Operating System (ROS). This package provides a service that allows users to send prompts to ChatGPT and receive responses in real-time.

## Overview

This package exposes a ROS service that allows for interaction with the ChatGPT API. Users can send prompts and receive generated responses, making it suitable for various applications, including conversational agents, Q&A systems, and robotic applications requiring natural language processing.

## Features

- Integrates with OpenAI's ChatGPT API.
- Provides a ROS service for querying the AI model.
- Supports customizable prompts and responses.

## Prerequisites

- ROS (Robot Operating System) installed.
- Python 3 installed.
- An active OpenAI API key. You can sign up at [OpenAI](https://platform.openai.com/signup).

## Installation

**1. Clone the repository**:

```bash
git clone https://github.com/toqqaa/chatgpt_ros.git
```

**2. Build the package** :
Ensure you are in your ROS workspace and run:

```bash
catkin_make
```

**3. Set up your OpenAI API key** :

Set your API key as an environment variable:

```bash
export OPENAI_API_KEY="your-api-key-here"
```

## Usage

1. **Start the ROS core** :
   Open a terminal and run:

```bash
roscore
```

2. **Launch the ChatGPT service** :
   In another terminal, navigate to your workspace and run:

   ```bash
    rosrun chatgpt_ros chatgpt_service.py 
   ```

**Call the service** :
You can call the service using the following command:

```bash
rosservice call /chatgpt_service "prompt: 'What is ROS and how does it work?'"
```

## Acknowledgments

* This package uses the OpenAI API. Check out the [OpenAI API documentation](https://platform.openai.com/docs/api-reference) for more information.
