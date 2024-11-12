from typing import Annotated

from datetime import datetime, timezone
from fastapi import Depends, FastAPI, HTTPException, Query
from fastapi.responses import JSONResponse
from sqlmodel import Field, Session, SQLModel, create_engine, select


class Message(SQLModel, table=True):
    message: str
    msg_timestamp: datetime = Field(default_factory=datetime.now)
    id: int = Field(default=None, primary_key=True)



sqlite_file_name = "database.db"
sqlite_url = f"sqlite:///{sqlite_file_name}"

connect_args = {"check_same_thread": False}
engine = create_engine(sqlite_url, connect_args=connect_args)


def create_db_and_tables():
    SQLModel.metadata.create_all(engine)


def get_session():
    with Session(engine) as session:
        yield session


SessionDep = Annotated[Session, Depends(get_session)]

app = FastAPI()


@app.on_event("startup")
def on_startup():
    create_db_and_tables()


@app.post("/messages/")
def send_message(msg: Message, session: SessionDep):
    session.add(msg)
    session.commit()
    session.refresh(msg)
    return None


@app.get("/messages/")
def read_messages(
    session: SessionDep
) -> list[Message]:
    # Get all messages
    stmt = select(Message)
    msgs = session.exec(stmt).all()
    # delete all messages
    resp = [msg.message for msg in msgs]
    for msg in msgs:
        session.delete(msg)
    session.commit()
    return JSONResponse(content=resp)
