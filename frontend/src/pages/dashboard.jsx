import React from 'react';
import Layout from '@theme/Layout';
import LearnerDashboard from '../components/LearnerDashboard';

export default function Dashboard() {
  return (
    <Layout
      title="Learner Dashboard"
      description="Track your progress, view assessment results, and manage your learning profile">
      <LearnerDashboard />
    </Layout>
  );
}
