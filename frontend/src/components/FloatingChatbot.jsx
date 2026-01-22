import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from './AuthProvider';
import { useTranslation } from './TranslationProvider';
import Lottie from 'lottie-react';
import robotAnimation from './lottie/ai-robot-animation.json';
import './FloatingChatbot.css';

// Greeting translations
const GREETINGS = {
  en: "👋 Hi! I'm your AI Assistant. Click me to chat!",
  ur: "👋 السلام علیکم! میں آپ کا AI اسسٹنٹ ہوں۔ بات کرنے کے لیے کلک کریں!",
  zh: "👋 你好！我是你的AI助手。点击我开始聊天！",
  es: "👋 ¡Hola! Soy tu Asistente de IA. ¡Haz clic para chatear!",
  ar: "👋 مرحباً! أنا مساعدك الذكي. انقر للدردشة!",
  hi: "👋 नमस्ते! मैं आपका AI सहायक हूं। चैट करने के लिए क्लिक करें!",
  fr: "👋 Bonjour! Je suis votre Assistant IA. Cliquez pour discuter!"
};

const INITIAL_MESSAGES = {
  en: "Hi! I'm your Physical AI assistant. Ask me anything about robotics, ROS 2, or the course content!",
  ur: "السلام علیکم! میں آپ کا فزیکل AI اسسٹنٹ ہوں۔ روبوٹکس، ROS 2، یا کورس کے بارے میں کچھ بھی پوچھیں!",
  zh: "你好！我是你的物理AI助手。问我任何关于机器人、ROS 2或课程内容的问题！",
  es: "¡Hola! Soy tu asistente de IA Física. ¡Pregúntame sobre robótica, ROS 2 o el contenido del curso!",
  ar: "مرحباً! أنا مساعدك للذكاء الاصطناعي الفيزيائي. اسألني أي شيء عن الروبوتات أو ROS 2!",
  hi: "नमस्ते! मैं आपका फिजिकल AI असिस्टेंट हूं। रोबोटिक्स, ROS 2 या कोर्स के बारे में कुछ भी पूछें!",
  fr: "Bonjour! Je suis votre assistant IA Physique. Posez-moi vos questions sur la robotique, ROS 2 ou le cours!"
};

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
  const { currentLang } = useTranslation();
  const [isOpen, setIsOpen] = useState(false);
  const [showGreeting, setShowGreeting] = useState(false);
  const [robotEntered, setRobotEntered] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: INITIAL_MESSAGES['en'],
    }
  ]);

  // Update initial message when language changes
  useEffect(() => {
    setMessages([{
      role: 'assistant',
      content: INITIAL_MESSAGES[currentLang] || INITIAL_MESSAGES['en'],
    }]);
  }, [currentLang]);
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);

  // Robot enters from right side and shows greeting
  useEffect(() => {
    const robotTimer = setTimeout(() => setRobotEntered(true), 500);
    const greetingTimer = setTimeout(() => {
      setShowGreeting(true);
      // Hide greeting after 8 seconds
      setTimeout(() => setShowGreeting(false), 8000);
    }, 1500);

    return () => {
      clearTimeout(robotTimer);
      clearTimeout(greetingTimer);
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
      {/* Full Robot with Greeting */}
      {!isOpen && (
        <div className={`robot-container ${robotEntered ? 'entered' : ''}`}>
          {/* Greeting Bubble */}
          {showGreeting && (
            <div className="chatbot-greeting">
              <span>{GREETINGS[currentLang] || GREETINGS['en']}</span>
            </div>
          )}

          {/* Lottie Robot Animation */}
          <button
            className="chatbot-float-btn"
            onClick={() => { setIsOpen(true); setShowGreeting(false); }}
            aria-label="Open AI Assistant"
          >
            <div className="lottie-robot-container">
              <Lottie
                animationData={robotAnimation}
                loop={true}
                autoplay={true}
                className="lottie-robot"
              />
            </div>

            <span className="pulse-ring"></span>
            <span className="pulse-ring delay"></span>
          </button>
        </div>
      )}

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
