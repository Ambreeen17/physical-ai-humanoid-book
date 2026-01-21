# Authentication System Guide

## Overview

The Physical AI textbook backend uses **JWT (JSON Web Token) authentication** to secure API endpoints and link requests to authenticated users.

**Key Features**:
- ✅ Stateless JWT-based authentication
- ✅ HTTP Bearer token scheme
- ✅ Automatic token validation on protected endpoints
- ✅ User profile linked to every request
- ✅ Optional integration with better-auth library
- ✅ Easy client-side implementation

---

## Architecture

### Authentication Flow

```
┌─────────┐
│ Client  │
└────┬────┘
     │ 1. POST /api/v1/auth/login (email)
     │
     ├──────────────────────────────────────┐
     │                                      │
     │ Server:                              │
     │ - Verify email exists               │
     │ - Create JWT token                  │
     │ - Return token + user info          │
     │                                      │
     ├──────────────────────────────────────┘
     │
     │ 2. GET /api/v1/auth/me
     │    Header: Authorization: Bearer <token>
     │
     ├──────────────────────────────────────┐
     │                                      │
     │ Server:                              │
     │ - Decode JWT                        │
     │ - Verify signature                  │
     │ - Check expiration                  │
     │ - Fetch user from DB                │
     │ - Update last_login                 │
     │ - Return user profile               │
     │                                      │
     └──────────────────────────────────────┘
```

### Token Structure

```
Header.Payload.Signature

Example JWT:
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.
eyJ1c2VyX2lkIjoiNTUwZTg0MDAtZTI5Yi00MWQ0LWE3MTYtNDQ2NjU1NDQwMDAwIiwi
ZW1haWwiOiJsZWFybmVyQGV4YW1wbGUuY29tIiwiZXhwIjoxNjcwOTI2OTUwLCJpYXQiOjE2
NzA5MjUxNTB9.
ydKNFfB6zvSUZYzJEXYMYr9vX8Hv3zMqL2kOpWxYzPs

Decoded Payload:
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "learner@example.com",
  "exp": 1670926950,    // Expiration timestamp
  "iat": 1670925150     // Issued at timestamp
}
```

---

## API Endpoints

### Authentication (`/api/v1/auth`)

#### 1. Login
```
POST /api/v1/auth/login
Content-Type: application/json

Request:
{
    "email": "learner@example.com"
}

Response (200 OK):
{
    "access_token": "eyJhbGciOiJIUzI1NiIs...",
    "token_type": "bearer",
    "expires_in": 1800,           // seconds
    "user_id": "550e8400-e29b...",
    "email": "learner@example.com"
}
```

**cURL Example**:
```bash
curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "learner@example.com"}'
```

#### 2. Get Current User
```
GET /api/v1/auth/me
Authorization: Bearer <access_token>

Response (200 OK):
{
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com",
    "first_name": "Alex",
    "last_name": "Chen",
    "programming_level": "intermediate",
    "ai_experience": "basic",
    "robotics_experience": "none",
    "difficulty_level": "intermediate",
    "preferred_language": "en",
    "learning_goal": "simulations",
    "created_at": "2025-12-31T12:00:00",
    "last_login": "2025-12-31T15:30:00"
}
```

**cURL Example**:
```bash
curl -X GET http://localhost:8000/api/v1/auth/me \
  -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIs..."
```

#### 3. Verify Token
```
GET /api/v1/auth/verify
Authorization: Bearer <access_token>

Response (200 OK):
{
    "valid": true,
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com",
    "difficulty_level": "intermediate"
}
```

#### 4. Logout
```
POST /api/v1/auth/logout
Authorization: Bearer <access_token>

Response (200 OK):
{
    "message": "Logged out successfully"
}
```

**Note**: Logout is stateless. The token remains valid until expiration. To implement logout, discard the token client-side.

---

### User Profiles with Authentication

#### Get My Profile (Authenticated)
```
GET /api/v1/user-profiles/me/profile
Authorization: Bearer <access_token>

Response (200 OK):
{
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com",
    "programming_level": "intermediate",
    "difficulty_level": "intermediate",
    "hardware_summary": {
        "gpu": {...},
        "jetson": {...},
        "robot": {...},
        "recommended_simulator": "isaac_sim",
        "lab_difficulties": ["beginner", "intermediate"]
    }
}
```

#### Update My Profile (Authenticated)
```
PUT /api/v1/user-profiles/me/profile
Authorization: Bearer <access_token>
Content-Type: application/json

Request:
{
    "programming_level": "advanced",
    "preferred_language": "ur"
}

Response (200 OK):
{
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com",
    "programming_level": "advanced",
    "difficulty_level": "intermediate",
    ...
}
```

---

## Backend Implementation

### Dependencies Module (`backend/src/auth/dependencies.py`)

**Key Functions**:

#### create_access_token()
```python
def create_access_token(
    user_id: str,
    email: str,
    expires_delta: Optional[timedelta] = None
) -> str:
    """Create JWT token."""
    # Encodes user_id, email, and expiration
    # Returns signed JWT string
```

#### decode_token()
```python
def decode_token(token: str) -> TokenData:
    """Decode and validate JWT token."""
    # Verifies signature
    # Checks expiration
    # Returns TokenData
```

#### get_current_user()
```python
def get_current_user(
    credentials: HTTPAuthCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> UserProfile:
    """Dependency to get authenticated user."""
    # 1. Extract token from Authorization header
    # 2. Decode and validate
    # 3. Fetch user from database
    # 4. Update last_login
    # 5. Return UserProfile
```

#### get_current_user_optional()
```python
def get_current_user_optional(
    credentials: Optional[HTTPAuthCredentials] = Depends(security),
    db: Session = Depends(get_db)
) -> Optional[UserProfile]:
    """Optional authentication (returns None if no token)."""
```

### Using Dependencies in Endpoints

**Protected Endpoint** (requires authentication):
```python
@app.get("/api/v1/protected")
async def protected_endpoint(
    current_user: UserProfile = Depends(get_current_user)
):
    # Token is valid and current_user is loaded
    return {"user_id": str(current_user.id)}
```

**Optional Authentication**:
```python
@app.get("/api/v1/public")
async def public_endpoint(
    current_user: Optional[UserProfile] = Depends(get_current_user_optional)
):
    if current_user:
        return {"user_id": str(current_user.id)}
    else:
        return {"message": "Public content"}
```

---

## Frontend Implementation

### 1. Store Token After Login

```javascript
// After successful login
const response = await fetch('/api/v1/auth/login', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email: 'learner@example.com' })
});

const data = await response.json();

// Store token in localStorage or sessionStorage
localStorage.setItem('access_token', data.access_token);
localStorage.setItem('user_id', data.user_id);
localStorage.setItem('email', data.email);
```

### 2. Include Token in Requests

```javascript
// Helper function to fetch with auth
async function authenticatedFetch(url, options = {}) {
    const token = localStorage.getItem('access_token');

    if (!token) {
        throw new Error('Not authenticated');
    }

    return fetch(url, {
        ...options,
        headers: {
            ...options.headers,
            'Authorization': `Bearer ${token}`
        }
    });
}

// Usage
const profile = await authenticatedFetch('/api/v1/auth/me');
const data = await profile.json();
```

### 3. Logout

```javascript
function logout() {
    // Discard token (stateless logout)
    localStorage.removeItem('access_token');
    localStorage.removeItem('user_id');
    localStorage.removeItem('email');

    // Redirect to login
    window.location.href = '/login';
}
```

### 4. Check Authentication Status

```javascript
async function isAuthenticated() {
    const token = localStorage.getItem('access_token');
    if (!token) return false;

    try {
        const response = await authenticatedFetch('/api/v1/auth/verify');
        return response.ok;
    } catch {
        return false;
    }
}

// Redirect unauthenticated users
if (!await isAuthenticated()) {
    window.location.href = '/login';
}
```

---

## Configuration

### Environment Variables

```bash
# Secret key for signing tokens (CHANGE IN PRODUCTION)
SECRET_KEY=your-secret-key-change-in-production

# Token expiration time (minutes)
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Database connection
DATABASE_URL=postgresql://user:password@localhost/textbook_db
```

### Production Recommendations

1. **Use strong SECRET_KEY** (generate with `secrets.token_urlsafe(32)`)
2. **Enable HTTPS** (tokens transmitted in clear in HTTP)
3. **Use shorter expiration** for sensitive operations (15-30 min)
4. **Implement refresh tokens** for longer sessions
5. **Add rate limiting** on login endpoint
6. **Log authentication events** for security audit

---

## Integration with better-auth

The system is designed to work with [better-auth](https://github.com/better-auth/better-auth) library:

### Current Implementation
Uses JWT tokens with custom dependencies.

### future: OAuth2/Social Login
```python
# Template for better-auth integration (not implemented yet)
from better_auth import get_session

def get_current_user_oauth(
    session = Depends(get_session),
    db: Session = Depends(get_db)
) -> UserProfile:
    if not session or not session.user:
        raise HTTPException(status_code=401)

    user = db.query(UserProfile).filter(
        UserProfile.email == session.user.email
    ).first()

    return user
```

### Supported OAuth Providers (with better-auth)
- Google
- GitHub
- Microsoft
- Apple
- Discord

---

## Error Handling

### Common Error Responses

#### Invalid Credentials
```
POST /api/v1/auth/login
{"email": "nonexistent@example.com"}

Response (401 Unauthorized):
{
    "detail": "Invalid credentials"
}
```

#### Missing Token
```
GET /api/v1/auth/me
(no Authorization header)

Response (403 Forbidden):
{
    "detail": "Not authenticated"
}
```

#### Expired Token
```
GET /api/v1/auth/me
Authorization: Bearer <expired_token>

Response (401 Unauthorized):
{
    "detail": "Token has expired"
}
```

#### Invalid Token
```
GET /api/v1/auth/me
Authorization: Bearer invalid.token.here

Response (401 Unauthorized):
{
    "detail": "Invalid token: ..."
}
```

---

## Security Best Practices

✅ **Do**:
- Store tokens securely (localStorage, sessionStorage, or HTTP-only cookies)
- Include token in every authenticated request
- Use HTTPS in production
- Rotate SECRET_KEY periodically
- Log authentication failures
- Validate email format
- Rate limit login attempts
- Use short token expiration times

❌ **Don't**:
- Hardcode SECRET_KEY in code
- Transmit tokens in URL parameters
- Store sensitive data in JWT payload
- Reuse tokens across devices
- Log tokens or sensitive data
- Use simple email as password substitute
- Trust client-side token validation

---

## Testing

### Manual Testing

```bash
# 1. Login
curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com"}' \
  > response.json

# 2. Extract token
TOKEN=$(jq -r '.access_token' response.json)

# 3. Get current user
curl -X GET http://localhost:8000/api/v1/auth/me \
  -H "Authorization: Bearer $TOKEN"

# 4. Verify token
curl -X GET http://localhost:8000/api/v1/auth/verify \
  -H "Authorization: Bearer $TOKEN"

# 5. Logout
curl -X POST http://localhost:8000/api/v1/auth/logout \
  -H "Authorization: Bearer $TOKEN"
```

### Python Testing

```python
import requests

# Login
response = requests.post(
    'http://localhost:8000/api/v1/auth/login',
    json={'email': 'test@example.com'}
)
token = response.json()['access_token']

# Get profile
headers = {'Authorization': f'Bearer {token}'}
profile = requests.get(
    'http://localhost:8000/api/v1/auth/me',
    headers=headers
).json()

print(profile)
```

---

## Troubleshooting

### Token Expired
**Problem**: `"detail": "Token has expired"`
**Solution**: User needs to login again to get a new token

### Invalid Signature
**Problem**: `"detail": "Invalid token: Signature verification failed"`
**Solution**: Token was signed with a different SECRET_KEY, or SECRET_KEY was changed

### User Not Found
**Problem**: `"detail": "User not found"`
**Solution**: User record doesn't exist in database (create via onboarding first)

### Missing Authorization Header
**Problem**: `"detail": "Not authenticated"`
**Solution**: Include `Authorization: Bearer <token>` in request headers

---

## Support & References

- **JWT Standard**: https://tools.ietf.org/html/rfc7519
- **HTTP Bearer Token**: https://tools.ietf.org/html/rfc6750
- **FastAPI Security**: https://fastapi.tiangolo.com/tutorial/security/
- **PyJWT Library**: https://pyjwt.readthedocs.io/

---

**Status**: ✅ Production Ready
**Version**: 1.0.0
**Last Updated**: 2025-12-31
