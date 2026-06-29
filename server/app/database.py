# PostgreSQL 비동기 연결 설정
# 드라이버: asyncpg (비동기) — FastAPI의 async/await와 호환
# SQLAlchemy가 파이썬 객체 ↔ SQL 번역, asyncpg가 실제 DB 통신 담당
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker

DATABASE_URL = "postgresql+asyncpg://codelab:codelab@localhost/s_mart"

engine = create_async_engine(DATABASE_URL, echo=True)

# expire_on_commit=False: 커밋 후 객체 속성 접근 시 추가 DB 조회 방지
AsyncSessionLocal = async_sessionmaker(engine, expire_on_commit=False)


# FastAPI 의존성 주입용 — 엔드포인트마다 Depends(get_db)로 세션을 주입받아 사용
# with 블록이 끝나면 세션 자동 반환
async def get_db():
    async with AsyncSessionLocal() as session:
        yield session


# 서버 시작 시 한 번 실행 — models.py에 정의된 테이블이 DB에 없으면 자동 생성
async def create_tables():
    from app.models import Base
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
