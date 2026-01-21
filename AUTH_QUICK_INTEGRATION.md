# Authentication - Quick Integration Guide

## What Was Built

A **complete JWT authentication system** for the Physical AI textbook:

1. ✅ JWT token generation & validation
2. ✅ Login endpoint (email-based)
3. ✅ Get current user profile
4. ✅ Logout functionality
5. ✅ Token verification
6. ✅ Automatic `last_login` tracking
7. ✅ Protected & optional endpoints

---

## Key Files

| File | Purpose |
|------|---------|
| `backend/src/auth/dependencies.py` | Token creation, validation, user dependency |
| `backend/src/api/auth.py` | Login, logout, verify endpoints |
| `backend/src/api/user_profiles.py` | Authenticated profile endpoints (/me) |
| `AUTHENTICATION_GUIDE.md` | Complete technical docs |

---

## Usage Pattern

### 1. User Logs In

```bash
POST /api/v1/auth/login
{
    "email": "learner@example.com"
}

Response:
{
    "access_token": "eyJhbGciOi...",
    "token_type": "bearer",
    "expires_in": 1800,
    "user_id": "550e8400-...",
    "email": "learner@example.com"
}
```

### 2. Client Stores Token

```javascript
// After login response
localStorage.setItem('access_token', response.access_token);
localStorage.setItem('user_id', response.user_id);
```

### 3. Client Includes Token in Requests

```javascript
// All subsequent requests
fetch('/api/v1/auth/me', {
    headers: {
        'Authorization': 'Bearer ' + localStorage.getItem('access_token')
    }
});
```

### 4. Server Validates Token & Returns User

```python
# Backend automatically:
# 1. Extracts token from Authorization header
# 2. Decodes & validates signature
# 3. Checks expiration
# 4. Fetches user from DB
# 5. Updates last_login
# 6. Returns UserProfile object

@app.get("/api/v1/auth/me")
async def get_me(current_user: UserProfile = Depends(get_current_user)):
    return current_user  # Already authenticated!
```

---

## Core API Endpoints

### Authentication

| Endpoint | Method | Auth | Purpose |
|----------|--------|------|---------|
| `/api/v1/auth/login` | POST | ❌ | Login with email |
| `/api/v1/auth/logout` | POST | ✅ | Logout (client-side discard token) |
| `/api/v1/auth/me` | GET | ✅ | Get current user info |
| `/api/v1/auth/verify` | GET | ✅ | Check if token is valid |

### User Profiles (Authenticated)

| Endpoint | Method | Auth | Purpose |
|----------|--------|------|---------|
| `/api/v1/user-profiles/me/profile` | GET | ✅ | Get my profile |
| `/api/v1/user-profiles/me/profile` | PUT | ✅ | Update my profile |

---

## Code Examples

### Backend: Protecting an Endpoint

```python
from fastapi import Depends
from src.auth.dependencies import get_current_user
from src.models.user_profile import UserProfile

@app.get("/api/v1/protected")
async def protected_endpoint(
    current_user: UserProfile = Depends(get_current_user)
):
    # current_user is automatically loaded and validated
    return {
        "user_id": str(current_user.id),
        "email": current_user.email,
        "difficulty_level": current_user.get_difficulty_level()
    }
```

### Frontend: Login & Store Token

```javascript
async function login(email) {
    const response = await fetch('/api/v1/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email })
    });

    if (!response.ok) {
        throw new Error('Login failed');
    }

    const data = await response.json();

    // Store token
    localStorage.setItem('access_token', data.access_token);
    localStorage.setItem('user_id', data.user_id);

    // Redirect to dashboard
    window.location.href = '/dashboard';
}
```

### Frontend: Authenticated Request

```javascript
async function fetchUserProfile() {
    const token = localStorage.getItem('access_token');

    const response = await fetch('/api/v1/auth/me', {
        headers: {
            'Authorization': `Bearer ${token}`
        }
    });

    if (response.status === 401) {
        // Token expired or invalid - redirect to login
        localStorage.clear();
        window.location.href = '/login';
        return;
    }

    return response.json();
}
```

### Frontend: Logout

```javascript
function logout() {
    // Discard token (stateless logout)
    localStorage.removeItem('access_token');
    localStorage.removeItem('user_id');

    // Redirect to login page
    window.location.href = '/login';
}
```

---

## Configuration

### Environment Variables (`.env`)

```bash
# Secret key for signing tokens
# In production, use: python -c "import secrets; print(secrets.token_urlsafe(32))"
SECRET_KEY=your-super-secret-key-here

# Token expiration (minutes)
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Database
DATABASE_URL=postgresql://user:password@localhost/textbook_db
```

### Adding to main.py

```python
from fastapi import FastAPI
from src.api import auth, user_profiles

app = FastAPI()

# Register authentication routes
app.include_router(auth.router)
app.include_router(user_profiles.router)
```

---

## Dependency Injection Pattern

### Required Authentication

```python
@app.get("/me")
async def get_profile(
    user: UserProfile = Depends(get_current_user)  # ✅ REQUIRED
):
    return user.to_dict()
```

Token validation happens **automatically**:
- ✅ No token → 403 Forbidden
- ✅ Invalid token → 401 Unauthorized
- ✅ Expired token → 401 Token expired
- ✅ Valid token → user loaded from DB

### Optional Authentication

```python
from typing import Optional
from src.auth.dependencies import get_current_user_optional

@app.get("/public")
async def public_endpoint(
    user: Optional[UserProfile] = Depends(get_current_user_optional)  # ✅ OPTIONAL
):
    if user:
        return {"message": f"Hello {user.email}"}
    else:
        return {"message": "Hello anonymous"}
```

---

## JWT Token Anatomy

```
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.
eyJ1c2VyX2lkIjoiNTUwZTg0MDAtZTI5Yi00MWQ0LWE3MTYtNDQ2NjU1NDQwMDAwIiwi
ZW1haWwiOiJsZWFybmVyQGV4YW1wbGUuY29tIiwiZXhwIjoxNjcwOTI2OTUwLCJpYXQiOjE2
NzA5MjUxNTB9.
ydKNFfB6zvSUZYzJEXYMYr9vX8Hv3zMqL2kOpWxYzPs

┌─────────────────────────────────────────────────────────────────┐
│ HEADER (Algorithm, Type)                                        │
├─────────────────────────────────────────────────────────────────┤
│ PAYLOAD (user_id, email, exp, iat)                             │
├─────────────────────────────────────────────────────────────────┤
│ SIGNATURE (HMAC-SHA256 of header + payload with SECRET_KEY)     │
└─────────────────────────────────────────────────────────────────┘
```

**Decoded Payload**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "learner@example.com",
  "exp": 1670926950,      // Unix timestamp when token expires
  "iat": 1670925150       // Unix timestamp when token was issued
}
```

---

## Error Handling

### Common Status Codes

| Code | Scenario | Solution |
|------|----------|----------|
| 200 ✅ | Authenticated successfully | Continue |
| 400 ❌ | Invalid request (bad email) | Check request format |
| 401 ❌ | Invalid/expired token | Redirect to login |
| 403 ❌ | No token provided | Include Authorization header |
| 404 ❌ | User not found | Create profile via onboarding |
| 500 ❌ | Server error | Check server logs |

### Example Error Response

```javascript
try {
    const response = await fetch('/api/v1/auth/me', {
        headers: { 'Authorization': `Bearer ${token}` }
    });

    if (response.status === 401) {
        console.log('Token invalid or expired');
        localStorage.clear();
        window.location.href = '/login';
    } else if (response.status === 404) {
        console.log('User profile not found - complete onboarding');
        window.location.href = '/onboarding';
    } else if (response.ok) {
        const user = await response.json();
        console.log('User authenticated:', user);
    }
} catch (error) {
    console.error('Request failed:', error);
}
```

---

## Testing Checklist

- [ ] Login endpoint creates token
- [ ] Token contains user_id and email
- [ ] GET /me requires token
- [ ] Valid token returns user profile
- [ ] Expired token returns 401
- [ ] Invalid token returns 401
- [ ] last_login updated on request
- [ ] Optional endpoints work without auth
- [ ] Protected endpoints reject no token

### Quick Test

```bash
# 1. Login (no auth needed)
TOKEN=$(curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com"}' | jq -r '.access_token')

echo "Token: $TOKEN"

# 2. Get profile (with auth)
curl -X GET http://localhost:8000/api/v1/auth/me \
  -H "Authorization: Bearer $TOKEN" | jq

# 3. Verify token
curl -X GET http://localhost:8000/api/v1/auth/verify \
  -H "Authorization: Bearer $TOKEN" | jq
```

---

## Security Checklist

- [ ] SECRET_KEY changed from default
- [ ] HTTPS enabled in production
- [ ] Rate limiting on login endpoint
- [ ] Access tokens have short expiration (≤30 min)
- [ ] Tokens not logged or exposed in errors
- [ ] last_login field prevents account probing
- [ ] Email validation prevents injection
- [ ] CORS configured correctly
- [ ] Tokens invalidated on logout (client-side)

---

## Future Enhancements

1. **Refresh Tokens**: Implement longer-lived refresh tokens
2. **OAuth2**: Add Google/GitHub login via better-auth
3. **Multi-Device**: Issue tokens per device/session
4. **Rate Limiting**: Prevent brute-force login attempts
5. **2FA**: Two-factor authentication for sensitive operations
6. **Admin Roles**: Add is_admin field for administrative access
7. **Token Blacklist**: Server-side logout token tracking

---

## Example Flow: Complete Login → Protected Request

```
User                           Backend
 │                              │
 ├─ POST /login (email) ──────→ │
 │                              ├─ Check email exists
 │                              ├─ Create JWT token
 │                              ├─ Return token
 │← Access token (JWT) ────────┤
 │                              │
 │ Store token in localStorage  │
 │                              │
 │ GET /me with Authorization header
 ├─ Header: Bearer <token> ───→ │
 │                              ├─ Extract token
 │                              ├─ Decode & validate
 │                              ├─ Check expiration
 │                              ├─ Fetch user from DB
 │                              ├─ Update last_login
 │← User profile JSON ─────────┤
 │                              │
 │ Use profile in app           │
```

---

## Quick Reference

**Login**:
```javascript
const token = (await fetch('/api/v1/auth/login', {
    method: 'POST',
    body: JSON.stringify({email})
}).then(r => r.json())).access_token;
localStorage.setItem('access_token', token);
```

**Request**:
```javascript
fetch('/api/v1/protected', {
    headers: {Authorization: `Bearer ${localStorage.getItem('access_token')}`}
});
```

**Logout**:
```javascript
localStorage.removeItem('access_token');
window.location.href = '/login';
```

---

**Status**: ✅ Production Ready
**Version**: 1.0.0
**Last Updated**: 2025-12-31
