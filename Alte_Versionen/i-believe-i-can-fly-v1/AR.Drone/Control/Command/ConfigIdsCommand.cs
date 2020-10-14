using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    class ConfigIdsCommand : ARCommand
    {
        private string sessionId, userId, appId;

        public ConfigIdsCommand(string sessionId, string userId, string appId)
            : base(CommandName.CONFIG_IDS)
        {
            this.sessionId = sessionId;
            this.userId = userId;
            this.appId = appId;
        }

        public override string GetCommandParams()
        {
            return string.Format("{0},{1},{2}", sessionId, userId, appId);
        }
    }
}
