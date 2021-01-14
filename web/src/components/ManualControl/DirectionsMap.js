import { Direction } from "components/ManualControl/Direction";

class DirectionsMap {
  _dirs;
  constructor() {
    this._dirs = [];
  }
  setDir(dir) {
    if(!this._dirs.includes(dir)) {
      this._dirs.push(dir);
    }
  };
  clearDir(dir) {
    const index = this._dirs.indexOf(dir);
    if (index > -1) {
      this._dirs.splice(index, 1);
    }
  }
  clearAll() {
    this._dirs = [];
  }
  getDir(dir) {
    return this._dirs.includes(dir);
  }
  anyDir() {
    return Boolean(this._dirs.length);
  }
  compare(dirs) {
    for(let dir in Direction) {
      if(dirs.getDir(Direction[dir]) !== this.getDir(Direction[dir])) {
        return false;
      }
    }
    return true;
  }
};

export default DirectionsMap;