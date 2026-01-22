import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from './AuthProvider';
import './FloatingChatbot.css';

// Demo responses for when backend is not available
const DEMO_RESPONSES = {
  'ros': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides tools, libraries, and conventions for building robot applications. Key concepts include nodes, topics, services, and actions.',
  'physical ai': 'Physical AI refers to artificial intelligence systems that interact with the physical world through embodied agents like robots. Unlike pure software AI, Physical AI must handle real-world physics, sensor noise, and actuator limitations.',
  'humanoid': 'Humanoid robots like the Unitree G1 are designed with a human-like body structure. The G1 features 23 degrees of freedom, force-feedback joints, and integrated sensors.',
  'kinematics': 'Kinematics describes robot motion without considering forces. Forward kinematics computes end-effector position from joint angles. Inverse kinematics solves for joint angles given a desired pose.',
  'chapter': 'This textbook has 16 chapters covering Physical AI from fundamentals to deployment. Start with Chapter 1 for an introduction to embodied intelligence!',
  'help': 'I can help you with questions about ROS 2, robotics, kinematics, control systems, computer vision, reinforcement learning, and more. Just ask!',
  'default': 'Great question! I can help with robotics topics like ROS 2, kinematics, control theory, computer vision, and AI for robots. Try asking about a specific chapter topic!'
};

const getDemoResponse = (query) => {
  const lowerQuery = query.toLowerCase();
  for (const [key, response] of Object.entries(DEMO_RESPONSES)) {
    if (key !== 'default' && lowerQuery.includes(key)) {
      return response;
    }
  }
  return DEMO_RESPONSES.default;
};

const FloatingChatbot = () => {
  const { user } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [showGreeting, setShowGreeting] = useState(true);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: "Hi! I'm your Physical AI assistant. Ask me anything about robotics, ROS 2, or the course content!",
    }
  ]);
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);

  // Auto show/hide greeting bubble
  useEffect(() => {
    const showTimer = setTimeout(() => setShowGreeting(true), 2000);
    const hideTimer = setTimeout(() => setShowGreeting(false), 8000);
    return () => {
      clearTimeout(showTimer);
      clearTimeout(hideTimer);
    };
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim()) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsTyping(true);

    try {
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: input,
          learner_id: user ? user.id : 'guest',
          conversation_history: messages.filter(m => m.role !== 'system')
        }),
      });

      if (!response.ok) {
        throw new Error('API request failed');
      }

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
    } catch (error) {
      console.error('Chat error:', error);
      // Fallback response if API fails
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: "I'm having trouble connecting to my brain server right now. Is the backend running?"
      }]);
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <>
      {/* Greeting Bubble */}
      {!isOpen && showGreeting && (
        <div className="chatbot-greeting">
          <span>👋 Hi! I'm your AI Assistant</span>
        </div>
      )}

      {/* Floating Button */}
      <button
        className={`chatbot-float-btn ${isOpen ? 'hidden' : ''}`}
        onClick={() => { setIsOpen(true); setShowGreeting(false); }}
        aria-label="Open AI Assistant"
      >
        <div className="float-btn-icon">
          <div className="robot-avatar">
            <div className="robot-head">
              <div className="robot-antenna"></div>
              <div className="robot-eyes">
                <div className="robot-eye left"></div>
                <div className="robot-eye right"></div>
              </div>
              <div className="robot-mouth"></div>
            </div>
          </div>
          <span className="pulse-ring"></span>
          <span className="pulse-ring delay"></span>
        </div>
      </button>

      {/* Chat Window */}
      <div className={`chatbot-popup ${isOpen ? 'open' : ''}`}>
        <div className="chatbot-popup-header">
          <div className="header-left">
            <span className="header-icon">🤖</span>
            <div className="header-info">
              <span className="header-title">Physical AI Assistant</span>
              <span className="header-status">
                <span className="status-dot"></span>
                Online
              </span>
            </div>
          </div>
          <button className="close-btn" onClick={() => setIsOpen(false)}>
            ✕
          </button>
        </div>

        <div className="chatbot-popup-messages">
          {messages.map((msg, idx) => (
            <div key={idx} className={`popup-message ${msg.role}`}>
              {msg.role === 'assistant' && <span className="msg-avatar">🤖</span>}
              <div className="msg-bubble">{msg.content}</div>
              {msg.role === 'user' && <span className="msg-avatar">👤</span>}
            </div>
          ))}
          {isTyping && (
            <div className="popup-message assistant">
              <span className="msg-avatar">🤖</span>
              <div className="msg-bubble typing">
                <span className="dot"></span>
                <span className="dot"></span>
                <span className="dot"></span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div className="chatbot-popup-input">
          <input
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask about robotics..."
            disabled={isTyping}
          />
          <button onClick={handleSend} disabled={isTyping || !input.trim()}>
            ➤
          </button>
        </div>

        <div className="chatbot-popup-footer">
          Powered by RAG • Demo Mode
        </div>
      </div>
    </>
  );
};

export default FloatingChatbot;
