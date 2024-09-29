#!/usr/bin/env python3

import os  # Import os to access environment variables
import rospy
from chatgpt_ros.srv import ChatPrompt, ChatPromptResponse
from openai import OpenAI

class ChatGPTService:
    def __init__(self):
        rospy.init_node('chatgpt_service_node')

        # Retrieve API key from environment variable
        self.api_key = os.getenv('OPENAI_API_KEY')

        if not self.api_key:
            rospy.logerr("OpenAI API key is not set. Please set the OPENAI_API_KEY environment variable.")
            raise ValueError("OpenAI API key is missing!")

        self.service = rospy.Service('chatgpt_service', ChatPrompt, self.handle_request)

        # Initialize the OpenAI client
        self.client = OpenAI(api_key=self.api_key)

        rospy.loginfo("ChatGPT Service Initialized")

    def handle_request(self, req):
        prompt = req.prompt  
        response = self.get_chatgpt_response(prompt)
        return ChatPromptResponse(response=response) 

    def get_chatgpt_response(self, prompt):
        try:
            rospy.loginfo(f"Calling OpenAI with prompt: {prompt}")
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}]
            )
            chatgpt_response = response.choices[0].message.content
            return chatgpt_response
        except Exception as e:
            rospy.logerr(f"API request failed: {e}")
            return "Error getting response from ChatGPT."

if __name__ == '__main__':
    try:
        service = ChatGPTService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
