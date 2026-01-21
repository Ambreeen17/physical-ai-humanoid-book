import React, { useState, useRef, useEffect } from 'react';
import './RAGChatbot.css';

// Demo responses for when backend is not available
const DEMO_RESPONSES = {
  'ros': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides tools, libraries, and conventions for building robot applications. Key concepts include nodes, topics, services, and actions. In this course, we use ROS 2 Humble for all hands-on labs.',
  'physical ai': 'Physical AI refers to artificial intelligence systems that interact with the physical world through embodied agents like robots. Unlike pure software AI, Physical AI must handle real-world physics, sensor noise, and actuator limitations. The Unitree G1 humanoid robot is our primary platform for learning Physical AI concepts.',
  'humanoid': 'Humanoid robots like the Unitree G1 are designed with a human-like body structure. The G1 features 23 degrees of freedom, force-feedback joints, and integrated sensors. This form factor enables research in locomotion, manipulation, and human-robot interaction.',
  'sim-to-real': 'The sim-to-real gap refers to differences between simulated and real-world robot behavior. We use NVIDIA Isaac Sim and Gazebo for simulation, then transfer learned policies to real hardware. Domain randomization and system identification help bridge this gap.',
  'kinematics': 'Kinematics describes robot motion without considering forces. Forward kinematics computes end-effector position from joint angles using DH parameters. Inverse kinematics solves for joint angles given a desired end-effector pose. Chapter 2 covers this in detail.',
  'sensor': 'The Unitree G1 includes RGB-D cameras, IMUs, joint encoders, and force/torque sensors. Sensor fusion combines data from multiple sensors to estimate robot state. The sensorimotor loop (Sense-Think-Act) is fundamental to all robot control.',
  'default': 'Great question! This topic is covered in the course materials. In demo mode, I can provide basic information about ROS 2, Physical AI, humanoid robots, kinematics, sensors, and sim-to-real transfer. For full AI-powered responses, please run the backend with Docker.'
};

// Check if query matches any demo topic
const getDemoResponse = (query) => {
  const lowerQuery = query.toLowerCase();
  for (const [key, response] of Object.entries(DEMO_RESPONSES)) {
    if (key !== 'default' && lowerQuery.includes(key)) {
      return response;
    }
  }
  return DEMO_RESPONSES.default;
};

/**
 * RAG-Powered Chatbot Component
 *
 * Features:
 * - Real-time chat with backend RAG service
 * - Demo mode when backend unavailable
 * - Context-aware responses using vector search (Qdrant)
 * - Conversation history display
 * - Loading states and error handling
 * - Mobile-responsive design
 */
const RAGChatbot = ({ learnerId }) => {
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: 'Hello! I\'m your Physical AI learning assistant. Ask me anything about embodied intelligence, ROS 2, humanoid robots, or the course content.',
      timestamp: new Date()
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Send message to backend RAG API
  const handleSendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = {
      role: 'user',
      content: input,
      timestamp: new Date()
    };

    // Add user message immediately
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      // Call backend RAG API
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userMessage.content,
          learner_id: learnerId,
          conversation_history: messages.slice(-5).map(m => ({ // Last 5 messages for context
            role: m.role,
            content: m.content
          }))
        })
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response
      const assistantMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
      setIsLoading(false);
    } catch (err) {
      console.error('Chat error:', err);

      // Fallback to demo mode when backend is unavailable
      const demoResponse = getDemoResponse(userMessage.content);
      const demoMessage = {
        role: 'assistant',
        content: `[Demo Mode] ${demoResponse}`,
        timestamp: new Date(),
        isDemo: true
      };
      setMessages(prev => [...prev, demoMessage]);
      setIsLoading(false);
    }
  };

  // Handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Clear conversation
  const handleClearChat = () => {
    setMessages([
      {
        role: 'assistant',
        content: 'Conversation cleared. How can I help you?',
        timestamp: new Date()
      }
    ]);
    setError(null);
  };

  return (
    <div className="rag-chatbot">
      <div className="chatbot-header">
        <div className="header-content">
          <span className="chatbot-icon">🤖</span>
          <div className="header-text">
            <h3>Physical AI Assistant</h3>
            <p className="status">
              {isLoading ? 'Thinking...' : 'Online'}
            </p>
          </div>
        </div>
        <button className="clear-btn" onClick={handleClearChat} title="Clear conversation">
          🗑️
        </button>
      </div>

      <div className="chatbot-messages">
        {messages.map((message, index) => (
          <div
            key={index}
            className={`message ${message.role} ${message.isError ? 'error' : ''}`}
          >
            <div className="message-avatar">
              {message.role === 'user' ? '👤' : '🤖'}
            </div>
            <div className="message-content">
              <div className="message-text">{message.content}</div>
              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  <strong>Sources:</strong>
                  <ul>
                    {message.sources.map((source, idx) => (
                      <li key={idx}>
                        <a href={source.url} target="_blank" rel="noopener noreferrer">
                          {source.title}
                        </a>
                        {source.section && <span className="source-section"> ({source.section})</span>}
                      </li>
                    ))}
                  </ul>
                </div>
              )}
              <div className="message-timestamp">
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="message assistant loading">
            <div className="message-avatar">🤖</div>
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {error && (
        <div className="chatbot-error">
          ⚠️ {error}
        </div>
      )}

      <div className="chatbot-input">
        <textarea
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask about ROS 2, embodied AI, sim-to-real gap, labs..."
          rows="2"
          disabled={isLoading}
        />
        <button
          className="send-btn"
          onClick={handleSendMessage}
          disabled={!input.trim() || isLoading}
        >
          {isLoading ? '⏳' : '➤'}
        </button>
      </div>

      <div className="chatbot-footer">
        <small>
          Powered by RAG (Retrieval-Augmented Generation) with Qdrant vector search
        </small>
      </div>
    </div>
  );
};

export default RAGChatbot;
