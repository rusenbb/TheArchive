# BLG317E Database Systems Project Documentation

## Course Information

- **Course**: BLG317E - Database Systems
- **Instructors**:
  - Erhan Biçer (bicer21@itu.edu.tr)
  - Sümeyye Öztürk (ozturks20@itu.edu.tr)

## Project Overview

This project involves developing a database system from concept to implementation, including a RESTful API interface. Students will work in teams to design, implement, and demonstrate a complete database solution.

## Project Timeline and Milestones

### Milestone 1: Project Proposal and Group Formation

**Due Date**: November 4, 2024

#### Requirements

1. **Group Formation**
   - Team size: 2-4 members
   - Clear definition of team member roles
2. **Project Proposal**
   - Maximum 1 page document
   - Must require at least 8 database tables with relationships
   - Include planned CRUD operations and queries
   - Justify database necessity and data management approach

#### Submission Guidelines

- Submit proposal form by the deadline
- Await instructor approval before proceeding

### Milestone 2: ER Diagram and Data Model Design

**Due Date**: TBD (After proposal approval)

#### Requirements

1. **Entity-Relationship Diagram**

   - Minimum 8 tables
   - Clear definition of non-key columns
   - Documented relationships (One-to-One, One-to-Many)

2. **Data Population Strategy**

   - Plan for dummy data generation
   - Real-world scenario simulation

3. **CRUD Operations Planning**
   - Detailed operation workflows
   - User interaction patterns
   - Query design considerations

### Milestone 3: Database Implementation and API Development

#### Database Requirements

1. **DBMS Selection**

   - Choose from:
     - MySQL
     - PostgreSQL
     - MongoDB
     - Other relational/non-relational databases

2. **Query Implementation**
   - Raw SQL queries required (No ORMs allowed)
   - Complex query implementation
   - Nested query support

#### API Development

1. **Technology Stack**

   - Recommended frameworks:
     - Node.js with Express
     - Flask/Django (Python)
     - Spring Boot (Java)
   - No auto-code generators allowed

2. **API Endpoints**

   - RESTful design required
   - Standard CRUD operations:
     ```
     GET    /items          # Retrieve data
     POST   /items          # Create data
     PUT    /items/:id      # Update data
     DELETE /items/:id      # Remove data
     ```

3. **Authentication**

   - JWT implementation required
   - User role management:
     - Admin access
     - Regular user access
   - Secure route protection

4. **Documentation**
   - Swagger/OpenAPI implementation required
   - Interactive API documentation
   - Clear endpoint descriptions
   - Request/response format documentation

### Milestone 4: Final Deliverables and Presentation

#### Required Deliverables

1. **Complete Documentation**

   - Project overview
   - ER diagram explanations
   - API endpoint documentation
   - Query implementations
   - Challenge solutions

2. **Live Demonstration**

   - Working system showcase
   - CRUD operation examples
   - Complex query demonstrations
   - API functionality presentation

3. **Technical Implementation**
   - Functional database system
   - Working API endpoints
   - Authentication system
   - Complex query support

#### Presentation Requirements

- Live system demonstration
- Architecture explanation
- Design decision justification
- Q&A session readiness

## Technical Requirements

### Database Implementation

- Raw SQL queries only
- No ORM usage
- Complete CRUD support
- Complex query capabilities

### API Development

```javascript
// Example API Structure
/api
  /auth
    POST /login    // User authentication
    POST /register // New user registration
  /items
    GET    /       // List items
    POST   /       // Create item
    GET    /:id    // Get specific item
    PUT    /:id    // Update item
    DELETE /:id    // Remove item
```

### Security Implementation

```javascript
// Example JWT Structure
{
  "header": {
    "alg": "HS256",
    "typ": "JWT"
  },
  "payload": {
    "user_id": "12345",
    "role": "admin",
    "exp": 1735689600
  }
}
```

## Development Tools

### Recommended Platforms

- Local development environment
- Cloud deployment options:
  - Heroku
  - AWS
  - Azure

### Testing Tools

- Postman
- curl
- Swagger UI

### Authentication Libraries

- Passport.js (Node.js)
- Flask-JWT-Extended (Flask)
- Django REST Framework JWT
- Spring Security (Java)

## Best Practices

### Database Design

- Proper normalization
- Efficient indexing
- Meaningful relationships
- Clear naming conventions

### API Design

- RESTful principles
- Proper status codes
- Consistent error handling
- Clear documentation

### Security

- Secure password handling
- Token-based authentication
- Role-based access control
- Input validation

## Evaluation Criteria

### Technical Assessment (70%)

- Database design (20%)
- API implementation (20%)
- Query complexity (15%)
- Security implementation (15%)

### Documentation & Presentation (30%)

- Code documentation (10%)
- API documentation (10%)
- Live demonstration (10%)

## Submission Guidelines

1. All code must be version controlled
2. Documentation in Markdown format
3. Working demonstration environment
4. Complete API documentation
5. Final presentation slides

## Support Resources

- Course materials
- Instructor office hours
- Online documentation
- Development community resources
