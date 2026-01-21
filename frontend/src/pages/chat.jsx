import React from 'react';
import Layout from '@theme/Layout';
import RAGChatbot from '../components/RAGChatbot';
import { TranslationProvider } from '../components/TranslationProvider';
import LanguageSwitcher from '../components/LanguageSwitcher';

export default function Chat() {
  // In production, get learner ID from authentication context
  const learnerId = 'demo-learner-123';

  return (
    <TranslationProvider>
      <Layout
        title="AI Assistant"
        description="Ask questions about Physical AI, ROS 2, robotics, and course content">
        <div style={{
          display: 'flex',
          justifyContent: 'flex-end',
          padding: '1rem 2rem',
          borderBottom: '1px solid #eee'
        }}>
          <LanguageSwitcher />
        </div>
        <div style={{ padding: '2rem 0' }}>
          <div style={{ maxWidth: '800px', margin: '0 auto', textAlign: 'center', marginBottom: '2rem' }}>
            <h1>🤖 Physical AI Learning Assistant</h1>
            <p style={{ fontSize: '1.125rem', color: 'var(--ifm-color-emphasis-700)' }}>
              Ask questions about embodied intelligence, ROS 2, humanoid robots, sim-to-real gap,
              and any content from the textbook. Powered by RAG with vector search.
            </p>
          </div>
          <RAGChatbot learnerId={learnerId} />
        </div>
      </Layout>
    </TranslationProvider>
  );
}
