# PortfolioVantage

## Authors

- [@rusenbb](https://github.com/rusenbb)
- [@littlestronomer](https://github.com/littlestronomer)
- [@anildervis](https://github.com/anildervis)
- [@itu-itis22-hussein22](https://github.com/itu-itis22-hussein22)

## Project Description

PortfolioVantage is a comprehensive financial portfolio management system designed to provide users with advanced tools for tracking, analyzing, and optimizing their investment portfolios. Built with TimescaleDB and PostgreSQL, the system offers robust time-series data management capabilities alongside traditional relational database features.

### Core Features

1. **Portfolio Management**

   - Real-time portfolio tracking
   - Multiple portfolio simulation ("what-if" scenarios)
   - Historical performance analysis
   - Asset allocation visualization

2. **Market Intelligence**

   - Integration of financial news from various sources
   - News-asset correlation tracking
   - User sentiment analysis on news items
   - Interactive commenting system

3. **Time-Series Analysis**

   - Historical price data management
   - Performance trending
   - Custom time period analysis
   - Price movement tracking

4. **User Interaction**
   - Secure user authentication
   - Customizable portfolio views
   - News interaction and commenting
   - Sentiment sharing (bullish/bearish indicators)

### Technical Architecture

#### Database Schema

1. **Users**

   - User account information
   - Authentication credentials
   - Profile settings

2. **Portfolios**

   - Portfolio configurations
   - Performance metrics
   - Investment strategies

3. **Assets**

   - Financial instrument details
   - Current market data
   - Asset classifications

4. **PortfolioAssets**

   - Holdings information
   - Purchase records
   - Quantity tracking

5. **PriceHistory**

   - Time-series price data
   - Historical performance metrics
   - Market trends

6. **News**

   - Financial news articles
   - Source information
   - Publication timestamps

7. **NewsAssets**

   - News-asset relationships
   - Relevance metrics
   - Impact analysis

8. **NewsInteractions**
   - User comments
   - Sentiment indicators
   - Interaction timestamps

### Technology Stack

- **Database**:

  - TimescaleDB for time-series data
  - PostgreSQL for relational data
  - Raw SQL queries for optimal performance

- **Backend**:

  - RESTful API architecture
  - JWT-based authentication
  - Role-based access control

- **API Documentation**:
  - Swagger/OpenAPI specification
  - Interactive documentation
  - Clear endpoint descriptions

### Key Objectives

1. **Data Integrity**

   - Maintain accurate portfolio information
   - Ensure reliable time-series data
   - Preserve user interaction history

2. **Performance**

   - Optimize time-series queries
   - Efficient portfolio calculations
   - Fast news retrieval and filtering

3. **Security**

   - Secure user authentication
   - Protected portfolio data
   - Safe transaction handling

4. **Usability**
   - Intuitive API design
   - Clear documentation
   - Consistent error handling

### Development Focus

- Implementing complex SQL queries without ORM
- Building efficient time-series data management
- Creating robust API endpoints
- Ensuring secure user authentication
- Developing comprehensive documentation

### Expected Outcomes

1. **For Users**

   - Better portfolio insights
   - Informed investment decisions
   - Enhanced market awareness

2. **For Developers**

   - Clean API architecture
   - Scalable database design
   - Maintainable codebase

3. **For System**
   - Robust performance
   - Reliable data management
   - Secure operations
