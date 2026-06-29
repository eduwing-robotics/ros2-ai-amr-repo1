# FastAPI 서버 진입점
# Customer Portal 전용 REST API + WebSocket
# Admin GUI, ROS2 패키지는 이 서버를 거치지 않고 DB에 직접 접근
import asyncio
import json
from datetime import datetime, timezone

import asyncpg

from fastapi import Depends, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy import select, text
from sqlalchemy.ext.asyncio import AsyncSession

from app.database import create_tables, get_db
from app.models import User, Order, OrderStatus, CancelReason, Product
from app.schemas import (
    LoginRequest, LoginResponse,
    SignupRequest,
    ProfileResponse, ProfileUpdate,
    OrderCreate, OrderResponse,
    ProductResponse,
)

app = FastAPI()

# Vite 개발 서버(5173)에서 오는 요청 허용
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── WebSocket 연결 관리 ───────────────────────────────────────────────────────

class ConnectionManager:
    """연결된 모든 브라우저에 이벤트를 broadcast하는 인메모리 관리자"""

    def __init__(self):
        self.active: list[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket):
        self.active.remove(ws)

    async def broadcast(self, message: dict):
        for ws in self.active:
            await ws.send_json(message)


manager = ConnectionManager()


# ── 서버 시작 ─────────────────────────────────────────────────────────────────

# asyncpg 전용 DSN (SQLAlchemy dialect prefix 없이)
_ASYNCPG_DSN = "postgresql://codelab:codelab@localhost/s_mart"


async def _listen_order_status():
    """PostgreSQL LISTEN order_status_updated → WebSocket broadcast.

    Task Manager / Fleet Manager가 NOTIFY order_status_updated 발행 시 수신.
    페이로드: {"order_id": X, "status": "processing|awaiting_pickup|delivered|cancelled",
               "cancel_reason": "no_pickup" (cancelled일 때만)}
    앱이 살아있는 동안 연결을 유지하며 이벤트를 대기.
    """
    conn = await asyncpg.connect(_ASYNCPG_DSN)

    async def on_notification(conn, pid, channel, payload):
        data = json.loads(payload)
        await manager.broadcast({"event": "order_status_updated", **data})

    async def on_location_notification(conn, pid, channel, payload):
        data = json.loads(payload)
        await manager.broadcast({"event": "location_updated", **data})

    await conn.add_listener("order_status_updated", on_notification)
    await conn.add_listener("location_updated", on_location_notification)

    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        await conn.remove_listener("order_status_updated", on_notification)
        await conn.remove_listener("location_updated", on_location_notification)
        await conn.close()


@app.on_event("startup")
async def startup():
    await create_tables()
    asyncio.create_task(_listen_order_status())


# ── 인증 ──────────────────────────────────────────────────────────────────────

@app.post("/auth/login", response_model=LoginResponse)
async def login(body: LoginRequest, db: AsyncSession = Depends(get_db)):
    # pgcrypto crypt(): 저장된 해시에서 salt 추출 → 입력값 동일 방식 해싱 → 비교
    result = await db.execute(
        text(
            "SELECT id, name, role FROM users "
            "WHERE email = :email AND password_hash = crypt(:password, password_hash)"
        ),
        {"email": body.email, "password": body.password},
    )
    row = result.fetchone()
    if not row:
        raise HTTPException(status_code=401, detail="이메일 또는 비밀번호가 올바르지 않습니다.")
    return LoginResponse(user_id=row.id, name=row.name, role=row.role)


@app.post("/auth/signup", response_model=LoginResponse, status_code=201)
async def signup(body: SignupRequest, db: AsyncSession = Depends(get_db)):
    exists = await db.execute(
        select(User).where(User.email == body.email)
    )
    if exists.scalar_one_or_none():
        raise HTTPException(status_code=409, detail="이미 사용 중인 이메일입니다.")

    result = await db.execute(
        text(
            "INSERT INTO users (email, password_hash, name, role) "
            "VALUES (:email, crypt(:password, gen_salt('bf')), :name, 'customer') "
            "RETURNING id, name, role"
        ),
        {"email": body.email, "password": body.password, "name": body.name},
    )
    row = result.fetchone()
    await db.commit()
    return LoginResponse(user_id=row.id, name=row.name, role=row.role)


# ── 사용자 프로필 ─────────────────────────────────────────────────────────────

@app.get("/users/{user_id}", response_model=ProfileResponse)
async def get_profile(user_id: int, db: AsyncSession = Depends(get_db)):
    user = await db.get(User, user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")
    return user


@app.patch("/users/{user_id}", response_model=ProfileResponse)
async def update_profile(user_id: int, body: ProfileUpdate, db: AsyncSession = Depends(get_db)):
    user = await db.get(User, user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    if body.name:
        user.name = body.name

    if body.new_password:
        if not body.current_password:
            raise HTTPException(status_code=400, detail="현재 비밀번호를 입력해 주세요.")
        verify = await db.execute(
            text("SELECT 1 FROM users WHERE id = :id AND password_hash = crypt(:pw, password_hash)"),
            {"id": user_id, "pw": body.current_password},
        )
        if not verify.fetchone():
            raise HTTPException(status_code=401, detail="현재 비밀번호가 올바르지 않습니다.")
        await db.execute(
            text("UPDATE users SET password_hash = crypt(:pw, gen_salt('bf')) WHERE id = :id"),
            {"pw": body.new_password, "id": user_id},
        )

    await db.commit()
    await db.refresh(user)
    return user


@app.delete("/users/{user_id}", status_code=204)
async def delete_account(user_id: int, db: AsyncSession = Depends(get_db)):
    user = await db.get(User, user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    active = await db.execute(
        select(Order).where(
            Order.user_id == user_id,
            Order.status.in_([OrderStatus.pending, OrderStatus.processing, OrderStatus.awaiting_pickup]),
        )
    )
    if active.scalar_one_or_none():
        raise HTTPException(status_code=409, detail="진행 중인 주문이 있어 탈퇴할 수 없습니다.")

    await db.execute(
        text("UPDATE tasks SET order_id = NULL WHERE order_id IN "
             "(SELECT id FROM orders WHERE user_id = :uid)"),
        {"uid": user_id},
    )
    await db.execute(
        text("DELETE FROM orders WHERE user_id = :uid"),
        {"uid": user_id},
    )
    await db.delete(user)
    await db.commit()


# ── 상품 ──────────────────────────────────────────────────────────────────────

@app.get("/products", response_model=list[ProductResponse])
async def list_products(db: AsyncSession = Depends(get_db)):
    result = await db.execute(
        text(
            "SELECT p.name, p.storage_type, "
            "COUNT(l.location_id) AS stock "
            "FROM products p "
            "LEFT JOIN locations l "
            "  ON l.product_name = p.name AND l.reserved_by IS NULL "
            "GROUP BY p.name, p.storage_type "
            "ORDER BY p.storage_type, p.name"
        )
    )
    return [
        ProductResponse(name=row.name, storage_type=row.storage_type, stock=row.stock)
        for row in result.fetchall()
    ]


# ── 주문 ──────────────────────────────────────────────────────────────────────

@app.post("/orders", response_model=OrderResponse, status_code=201)
async def create_order(body: OrderCreate, db: AsyncSession = Depends(get_db)):
    user = await db.get(User, body.user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    product = await db.get(Product, body.product_name)
    if not product:
        raise HTTPException(status_code=404, detail="상품을 찾을 수 없습니다.")

    order = Order(
        user_id=body.user_id,
        product_name=body.product_name,
        status=OrderStatus.pending,
        created_at=datetime.now(timezone.utc),
    )
    db.add(order)
    await db.flush()

    payload = json.dumps({"order_id": order.id})
    await db.execute(text(f"NOTIFY new_order, '{payload}'"))
    await db.commit()
    await db.refresh(order)

    await manager.broadcast({"event": "order_created", "order_id": order.id})
    return order


@app.get("/orders/me", response_model=list[OrderResponse])
async def my_orders(user_id: int, db: AsyncSession = Depends(get_db)):
    result = await db.execute(
        select(Order)
        .where(Order.user_id == user_id)
        .order_by(Order.created_at.desc())
    )
    return result.scalars().all()


@app.get("/orders/{order_id}", response_model=OrderResponse)
async def get_order(order_id: int, db: AsyncSession = Depends(get_db)):
    order = await db.get(Order, order_id)
    if not order:
        raise HTTPException(status_code=404, detail="주문을 찾을 수 없습니다.")
    return order


@app.post("/orders/{order_id}/cancel", response_model=OrderResponse)
async def cancel_order(order_id: int, db: AsyncSession = Depends(get_db)):
    order = await db.get(Order, order_id)
    if not order:
        raise HTTPException(status_code=404, detail="주문을 찾을 수 없습니다.")

    if order.status not in (OrderStatus.pending, OrderStatus.processing):
        raise HTTPException(status_code=409, detail="취소할 수 없는 상태입니다.")

    order.status = OrderStatus.cancelled
    order.cancel_reason = CancelReason.user
    await db.commit()
    await db.refresh(order)

    await manager.broadcast({"event": "order_cancelled", "order_id": order.id})
    return order


# ── WebSocket ─────────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(ws)
