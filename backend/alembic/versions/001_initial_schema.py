"""Initial schema with chapters, labs, assessments, learner profiles

Revision ID: 001
Revises:
Create Date: 2026-01-02 12:00:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '001'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create chapters table
    op.create_table(
        'chapters',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('number', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(length=255), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('status', sa.String(length=50), nullable=False),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.Column('updated_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('number')
    )
    op.create_index('ix_chapters_number', 'chapters', ['number'])

    # Create labs table
    op.create_table(
        'labs',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(length=255), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('difficulty', sa.String(length=50), nullable=False),
        sa.Column('estimated_time_minutes', sa.Integer(), nullable=False),
        sa.Column('docker_image', sa.String(length=255), nullable=True),
        sa.Column('test_script_path', sa.String(length=500), nullable=True),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.Column('updated_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index('ix_labs_chapter_id', 'labs', ['chapter_id'])

    # Create assessments table
    op.create_table(
        'assessments',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(length=255), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('assessment_type', sa.String(length=50), nullable=False),
        sa.Column('difficulty', sa.String(length=50), nullable=False),
        sa.Column('max_score', sa.Integer(), nullable=False),
        sa.Column('content', postgresql.JSON(astext_type=sa.Text()), nullable=False),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.Column('updated_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index('ix_assessments_chapter_id', 'assessments', ['chapter_id'])
    op.create_index('ix_assessments_type', 'assessments', ['assessment_type'])

    # Create learner_profiles table
    op.create_table(
        'learner_profiles',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('learner_id', sa.String(length=255), nullable=False),
        sa.Column('python_score', sa.Integer(), nullable=False),
        sa.Column('ml_score', sa.Integer(), nullable=False),
        sa.Column('robotics_score', sa.Integer(), nullable=False),
        sa.Column('ros_score', sa.Integer(), nullable=False),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.Column('updated_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('learner_id')
    )
    op.create_index('ix_learner_profiles_learner_id', 'learner_profiles', ['learner_id'])

    # Create assessment_results table
    op.create_table(
        'assessment_results',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('learner_profile_id', sa.Integer(), nullable=False),
        sa.Column('assessment_id', sa.Integer(), nullable=False),
        sa.Column('score', sa.Float(), nullable=False),
        sa.Column('max_score', sa.Float(), nullable=False),
        sa.Column('passed', sa.Boolean(), nullable=False),
        sa.Column('feedback', sa.Text(), nullable=True),
        sa.Column('raw_submission', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('created_at', sa.DateTime(), server_default=sa.text('now()'), nullable=False),
        sa.ForeignKeyConstraint(['assessment_id'], ['assessments.id'], ondelete='CASCADE'),
        sa.ForeignKeyConstraint(['learner_profile_id'], ['learner_profiles.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index('ix_assessment_results_learner_profile_id', 'assessment_results', ['learner_profile_id'])
    op.create_index('ix_assessment_results_assessment_id', 'assessment_results', ['assessment_id'])


def downgrade() -> None:
    # Drop tables in reverse order (respecting foreign keys)
    op.drop_index('ix_assessment_results_assessment_id', table_name='assessment_results')
    op.drop_index('ix_assessment_results_learner_profile_id', table_name='assessment_results')
    op.drop_table('assessment_results')

    op.drop_index('ix_learner_profiles_learner_id', table_name='learner_profiles')
    op.drop_table('learner_profiles')

    op.drop_index('ix_assessments_type', table_name='assessments')
    op.drop_index('ix_assessments_chapter_id', table_name='assessments')
    op.drop_table('assessments')

    op.drop_index('ix_labs_chapter_id', table_name='labs')
    op.drop_table('labs')

    op.drop_index('ix_chapters_number', table_name='chapters')
    op.drop_table('chapters')
